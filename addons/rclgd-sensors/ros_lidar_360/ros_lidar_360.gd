extends Node3D
class_name RosLidar360

# --- Constants ---
const FACE_CAPTURE_EFFECT_CLASS = preload("res://addons/rclgd-sensors/ros_lidar_360/lidar_360_compositor.gd")

# --- Configuration ---
@export var horizontal_resolution: int = 1024
@export var vertical_resolution: int = 64
@export var vertical_fov_min: float = -45.0 # Degrees
@export var vertical_fov_max: float = 45.0 # Degrees
@export var min_range: float = 0.1 # Meters
@export var max_range: float = 100.0 # Meters
@export var noise_std_dev: float = 0.005 # Standard deviation for Gaussian noise in meters
@export var face_resolution: int = 256 # Resolution of each cubemap face
@export var publish_rate: float = 10.0
@export var lidar_topic: String = "~/lidar_360"
@export var frame_id: String = "~lidar_360"
@export var parent_frame_id: String = "~base_link"

# --- Internal Nodes ---
var _viewports: Array[SubViewport] = []
var _cameras: Array[Camera3D] = []
var _remote_syncs: Array[RemoteTransform3D] = []

# --- Resource Retention (Crucial for RIDs) ---
var _compositors: Array[Compositor] = []
var _effects: Array[Lidar360FaceCompositorEffect] = []

# --- ROS Components ---
var _node: RosNode
var _lidar_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster
var _ros_timer: RosTimer

# --- Rendering Components ---
var _rd: RenderingDevice
var _face_textures: Array[RID] = []
var _output_texture_rid: RID
var is_sampling: bool = false
var _cached_msg: RosSensorMsgsPointCloud2
var _current_stamp: RosMsg

var _stitch_shader: RID
var _stitch_pipeline: RID

func _ready() -> void:
	# 1. Setup hardware rendering
	_rd = RenderingServer.get_rendering_device()
	_create_textures()
	_init_stitch_shader()
	
	# 2. Pipeline hierarchy (Viewports and cameras)
	_setup_rendering_pipeline()

	# 3. ROS 2 Init
	_node = RosNode.new()
	_node.init(name.to_snake_case())
	_lidar_pub = _node.create_publisher(lidar_topic, "sensor_msgs/msg/PointCloud2")
	_tf_broadcaster = _node.create_tf_broadcaster()

	# 4. Start loop
	_prepare_msg_template()
	var interval: float = 1.0 / publish_rate
	_ros_timer = _node.create_timer(interval, _on_timer_timeout)

func _setup_rendering_pipeline() -> void:
	# Rotations for the 6 faces (+X, -X, +Y, -Y, +Z, -Z)
	var rotations = [
		Vector3(0, deg_to_rad(-90), 0),  # +X (Right)
		Vector3(0, deg_to_rad(90), 0),   # -X (Left)
		Vector3(deg_to_rad(90), 0, 0),   # +Y (Up)
		Vector3(deg_to_rad(-90), 0, 0),  # -Y (Down)
		Vector3(0, deg_to_rad(180), 0),  # +Z (Back)
		Vector3(0, 0, 0),                # -Z (Forward)
	]
	
	var names = ["Right", "Left", "Up", "Down", "Back", "Forward"]

	for i in range(6):
		# Create SubViewport
		var vp = SubViewport.new()
		vp.name = "LidarViewport_" + names[i]
		vp.size = Vector2i(face_resolution, face_resolution)
		vp.render_target_update_mode = SubViewport.UPDATE_DISABLED
		add_child(vp)
		_viewports.append(vp)
		
		# Create Camera3D
		var cam = Camera3D.new()
		cam.name = "LidarCamera_" + names[i]
		cam.fov = 90.0
		cam.near = min_range
		cam.far = max_range
		vp.add_child(cam)
		_cameras.append(cam)
		
		# Setup Compositor Effect on Camera
		var effect = FACE_CAPTURE_EFFECT_CLASS.new()
		effect.target_tex = _face_textures[i]
		effect.texture_size = Vector2(face_resolution, face_resolution)
		_effects.append(effect)
		
		var comp = Compositor.new()
		comp.compositor_effects = [effect]
		_compositors.append(comp)
		cam.compositor = comp

		# Setup RemoteTransform3D to sync the camera to THIS node
		var sync = RemoteTransform3D.new()
		sync.name = "LidarSync_" + names[i]
		add_child(sync)
		sync.remote_path = sync.get_path_to(cam)
		sync.rotation = rotations[i]
		_remote_syncs.append(sync)

func _create_textures() -> void:
	# 6 face textures (RGBA32F: R = depth, G = intensity)
	for i in range(6):
		var fmt : RDTextureFormat = RDTextureFormat.new()
		fmt.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT 
		fmt.width = face_resolution
		fmt.height = face_resolution
		fmt.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | \
						 RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT | \
						 RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
		_face_textures.append(_rd.texture_create(fmt, RDTextureView.new()))
		
	# Output texture (RGBA32F: RGB = ROS Point, A = intensity)
	var out_fmt : RDTextureFormat = RDTextureFormat.new()
	out_fmt.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT 
	out_fmt.width = horizontal_resolution
	out_fmt.height = vertical_resolution
	out_fmt.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | \
						 RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT | \
						 RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
	_output_texture_rid = _rd.texture_create(out_fmt, RDTextureView.new())

func _init_stitch_shader() -> void:
	var shader_file: RDShaderFile = load("res://addons/rclgd-sensors/ros_lidar_360/LidarStitch.glsl")
	var shader_spirv: RDShaderSPIRV = shader_file.get_spirv()
	_stitch_shader = _rd.shader_create_from_spirv(shader_spirv)
	if _stitch_shader.is_valid():
		_stitch_pipeline = _rd.compute_pipeline_create(_stitch_shader)

func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd:
		# Free texture RIDs
		for tex in _face_textures:
			if tex.is_valid():
				_rd.free_rid(tex)
		if _output_texture_rid.is_valid():
			_rd.free_rid(_output_texture_rid)
		
		# Free shader
		if _stitch_shader.is_valid():
			_rd.free_rid(_stitch_shader)

func _on_timer_timeout():
	if is_sampling or not _output_texture_rid.is_valid(): return
		
	is_sampling = true
	_current_stamp = _node.now()
	
	# Update TF before render
	_tf_broadcaster.send_transform(global_transform, frame_id, parent_frame_id, false)

	# Trigger Render on all 6 viewports
	for vp in _viewports:
		vp.render_target_update_mode = SubViewport.UPDATE_ONCE
	
	# WAIT for the frame to be drawn before requesting data
	if not RenderingServer.frame_post_draw.is_connected(_on_frame_drawn):
		RenderingServer.frame_post_draw.connect(_on_frame_drawn, CONNECT_ONE_SHOT)

func _on_frame_drawn():
	# Serialize parameters on the main thread
	var params_bytes = _get_params_byte_array()
	
	# Dispatch stitching shader on the render thread
	RenderingServer.call_on_render_thread(_dispatch_stitching.bind(params_bytes))

func _dispatch_stitching(params_bytes: PackedByteArray) -> void:
	if not _stitch_shader.is_valid() or not _stitch_pipeline.is_valid() or not _output_texture_rid.is_valid():
		is_sampling = false
		return
		
	# 1. Uniforms for the 6 face textures (bindings 0 to 5)
	var uniforms: Array[RDUniform] = []
	for i in range(6):
		var u = RDUniform.new()
		u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		u.binding = i
		u.add_id(_face_textures[i])
		uniforms.append(u)
		
	# 2. Uniform for the Params Buffer (binding 6)
	var params_buffer = _rd.storage_buffer_create(params_bytes.size(), params_bytes)
	var params_uniform = RDUniform.new()
	params_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
	params_uniform.binding = 6
	params_uniform.add_id(params_buffer)
	uniforms.append(params_uniform)
	
	# 3. Uniform for the Output Texture (binding 7)
	var out_uniform = RDUniform.new()
	out_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	out_uniform.binding = 7
	out_uniform.add_id(_output_texture_rid)
	uniforms.append(out_uniform)
	
	# Create Uniform Set
	var uniform_set = _rd.uniform_set_create(uniforms, _stitch_shader, 0)
	
	# Calculate groups (assuming local_size_x/y = 8 in the shader)
	var x_groups = (horizontal_resolution - 1) / 8 + 1
	var y_groups = (vertical_resolution - 1) / 8 + 1
	
	# Dispatch compute list
	var compute_list = _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(compute_list, _stitch_pipeline)
	_rd.compute_list_bind_uniform_set(compute_list, uniform_set, 0)
	_rd.compute_list_dispatch(compute_list, x_groups, y_groups, 1)
	_rd.compute_list_end()
	
	# Request data asynchronously (sequenced after the compute commands)
	var err = _rd.texture_get_data_async(_output_texture_rid, 0, _on_texture_data_ready)
	if err != OK:
		is_sampling = false
	
	# Cleanup transient RIDs
	_rd.free_rid(uniform_set)
	_rd.free_rid(params_buffer)

func _on_texture_data_ready(raw_bytes: PackedByteArray):
	is_sampling = false
	if raw_bytes.is_empty(): return 
	
	_cached_msg.header.stamp = _current_stamp
	_cached_msg.width = raw_bytes.size() / 16
	_cached_msg.row_step = raw_bytes.size()
	_cached_msg.data = raw_bytes 
	
	_lidar_pub.publish(_cached_msg)

func _get_params_byte_array() -> PackedByteArray:
	var float_array = PackedFloat32Array()
	
	# 1. Output resolution
	float_array.append(float(horizontal_resolution))
	float_array.append(float(vertical_resolution))
	
	# 2. Seed and noise std dev
	float_array.append(randf())
	float_array.append(noise_std_dev)
	
	# 3. Vertical FOV bounds (converted to radians)
	float_array.append(deg_to_rad(vertical_fov_min))
	float_array.append(deg_to_rad(vertical_fov_max))
	
	# 4. Range bounds
	float_array.append(min_range)
	float_array.append(max_range)
	
	# 5. Inverse Projection Matrices (6 faces)
	for i in range(6):
		var inv_proj = _effects[i].inv_proj_matrix
		float_array.append_array(_serialize_projection(inv_proj))
		
	# 6. Camera Transforms (relative to Lidar node)
	for i in range(6):
		var trans = _remote_syncs[i].transform
		float_array.append_array(_serialize_transform(trans))
		
	# 7. Inverse Camera Transforms
	for i in range(6):
		var trans = _remote_syncs[i].transform
		var inv_trans = trans.affine_inverse()
		float_array.append_array(_serialize_transform(inv_trans))
		
	return float_array.to_byte_array()

func _serialize_projection(proj: Projection) -> PackedFloat32Array:
	return PackedFloat32Array([
		proj.x.x, proj.x.y, proj.x.z, proj.x.w,
		proj.y.x, proj.y.y, proj.y.z, proj.y.w,
		proj.z.x, proj.z.y, proj.z.z, proj.z.w,
		proj.w.x, proj.w.y, proj.w.z, proj.w.w,
	])

func _serialize_transform(t: Transform3D) -> PackedFloat32Array:
	return PackedFloat32Array([
		t.basis.x.x, t.basis.x.y, t.basis.x.z, 0.0,
		t.basis.y.x, t.basis.y.y, t.basis.y.z, 0.0,
		t.basis.z.x, t.basis.z.y, t.basis.z.z, 0.0,
		t.origin.x,  t.origin.y,  t.origin.z,  1.0,
	])

func _prepare_msg_template():
	_cached_msg = RosSensorMsgsPointCloud2.new()
	_cached_msg.height = 1
	_cached_msg.is_dense = true
	_cached_msg.point_step = 16 
	_cached_msg.header.frame_id = frame_id
	_cached_msg.fields = [
		_create_field("x", 0, 7), _create_field("y", 4, 7),
		_create_field("z", 8, 7), _create_field("intensity", 12, 7)
	]

func _create_field(fname: String, offset: int, datatype: int) -> RosSensorMsgsPointField:
	var f = RosSensorMsgsPointField.new()
	f.name = fname; f.offset = offset; f.datatype = datatype; f.count = 1
	return f
