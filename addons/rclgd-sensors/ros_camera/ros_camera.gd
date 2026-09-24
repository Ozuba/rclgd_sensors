extends Node3D
class_name RosCamera

const DEPTH_EFFECT_CLASS = preload("res://addons/rclgd-sensors/ros_camera/depth_compositor.gd")

# --- Configuration ---
@export_group("Sensor Settings")
@export var resolution: Vector2i = Vector2i(640, 480)
@export var fov: float = 75.0
@export var near: float = 0.05
@export var far: float = 150.0
@export_flags_3d_render var cull_mask: int = 1048575
@export var publish_depth: bool = false

@export_group("ROS 2 Settings")
@export var ros_namespace : String = ""
@export var publish_rate: float = 15.0
@export var frame_id: String = "camera_link"
@export var parent_frame_id: String = "base_link"
@export var optical_frame_id: String = "camera_optical"

# --- Internal Nodes ---
var _viewport: SubViewport
var _camera: Camera3D
var _remote_transform: RemoteTransform3D # The bridge node

# --- ROS Components ---
var _node: RosNode
var _camera_pub: RosPublisher
var _camera_info_pub: RosPublisher
var _depth_pub: RosPublisher
var _depth_info_pub: RosPublisher
var _tf_broadcaster: RosTfBroadcaster
var _ros_timer: RosTimer

# --- Internal Variables ---
var _cached_image_msg: RosSensorMsgsImage
var _cached_info_msg: RosSensorMsgsCameraInfo
var _cached_depth_msg: RosSensorMsgsImage
var _cached_depth_info_msg: RosSensorMsgsCameraInfo
var rd: RenderingDevice
var is_requesting: bool = false
var _pending_color: bool = false
var _pending_depth: bool = false
var _resolved_optical_frame: String
var _current_stamp: RosMsg

# --- Depth Capture (Compute Shader) Resources ---
var _depth_texture_rid: RID
var _depth_shader: RID
var _depth_pipeline: RID
var _depth_effect: DepthCaptureCompositorEffect
var _depth_compositor: Compositor

var optical_tf = Transform3D(Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0)).orthonormalized(), Vector3.ZERO)

func _ready() -> void:
	_setup_internal_nodes()
	Vector3.FORWARD
	_node = RosNode.new()
	_node.init(name.to_snake_case(),ros_namespace.to_snake_case())
	
	_camera_pub = _node.create_publisher("~/image_raw", "sensor_msgs/msg/Image")
	_camera_info_pub = _node.create_publisher("~/camera_info", "sensor_msgs/msg/CameraInfo")
	if publish_depth:
		_depth_pub = _node.create_publisher("~/depth/image_raw", "sensor_msgs/msg/Image")
		_depth_info_pub = _node.create_publisher("~/depth/camera_info", "sensor_msgs/msg/CameraInfo")
	_tf_broadcaster = _node.create_tf_broadcaster()
	_resolved_optical_frame = _node.resolve_frame(optical_frame_id)

	rd = RenderingServer.get_rendering_device()

	if publish_depth:
		_setup_depth_capture()

	_cache_message_templates(_camera)
	
	# Initial TF publish
	_tf_broadcaster.send_transform(optical_tf, optical_frame_id, frame_id, true)
	
	var interval: float = 1.0 / publish_rate
	_ros_timer = _node.create_timer(interval, _request_capture)

func _setup_internal_nodes() -> void:
	# 1. Create the Viewport (The rendering container)
	_viewport = SubViewport.new()
	_viewport.name = "RosViewport"
	_viewport.size = resolution
	_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	
	# Optimizations
	_viewport.positional_shadow_atlas_size = 0
	_viewport.screen_space_aa = SubViewport.SCREEN_SPACE_AA_DISABLED
	_viewport.use_debanding = false
	_viewport.use_hdr_2d = false
	
	add_child(_viewport)
	
	# 2. Create the Camera (Inside the viewport)
	_camera = Camera3D.new()
	_camera.name = "RosCamera3D"
	_camera.fov = fov
	_camera.near = near
	_camera.far = far
	_camera.cull_mask = cull_mask
	_viewport.add_child(_camera)

	# 3. Create the RemoteTransform3D (The sync bridge)
	# This ensures the internal camera follows this Node3D's movement
	_remote_transform = RemoteTransform3D.new()
	_remote_transform.name = "CameraSync"
	add_child(_remote_transform)
	
	# Link the bridge to the camera
	_remote_transform.remote_path = _remote_transform.get_path_to(_camera)

func _setup_depth_capture() -> void:
	# 1. Output texture (R32F: linear depth in meters along the optical axis, 0 = no return)
	var fmt: RDTextureFormat = RDTextureFormat.new()
	fmt.format = RenderingDevice.DATA_FORMAT_R32_SFLOAT
	fmt.width = resolution.x
	fmt.height = resolution.y
	fmt.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | \
					 RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT | \
					 RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
	_depth_texture_rid = rd.texture_create(fmt, RDTextureView.new())

	# 2. Compute shader that linearizes the hardware depth buffer for this camera's render pass
	var shader_file: RDShaderFile = load("res://addons/rclgd-sensors/ros_camera/DepthCapture.glsl")
	var spirv: RDShaderSPIRV = shader_file.get_spirv()
	_depth_shader = rd.shader_create_from_spirv(spirv)
	if _depth_shader.is_valid():
		_depth_pipeline = rd.compute_pipeline_create(_depth_shader)

	# 3. Compositor effect that dispatches the shader during the camera's own render pass
	_depth_effect = DEPTH_EFFECT_CLASS.new()
	_depth_effect.target_tex = _depth_texture_rid
	_depth_effect.shader = _depth_shader
	_depth_effect.pipeline = _depth_pipeline

	_depth_compositor = Compositor.new()
	_depth_compositor.compositor_effects = [_depth_effect]
	_camera.compositor = _depth_compositor

func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and rd:
		if _depth_texture_rid.is_valid():
			rd.free_rid(_depth_texture_rid)
		if _depth_shader.is_valid():
			rd.free_rid(_depth_shader)

func _request_capture() -> void:
	if not is_visible_in_tree() or is_requesting: return

	is_requesting = true
	_pending_color = true
	_pending_depth = publish_depth
	_current_stamp = _node.now()

	# Sync the TF right before we render to ensure the "photo" is taken at the correct spot
	# Use the exact same stamp as the image message to prevent synchronization lag
	_tf_broadcaster.send_transform(transform, frame_id, parent_frame_id, true)

	_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE

	if not RenderingServer.frame_post_draw.is_connected(_on_frame_drawn):
		RenderingServer.frame_post_draw.connect(_on_frame_drawn, CONNECT_ONE_SHOT)

func _on_frame_drawn() -> void:
	var tex = _viewport.get_texture()
	var rid = RenderingServer.texture_get_rd_texture(tex.get_rid())

	if rid.is_valid():
		rd.texture_get_data_async(rid, 0, _on_data_received)
	else:
		_pending_color = false

	if publish_depth:
		if _depth_texture_rid.is_valid():
			rd.texture_get_data_async(_depth_texture_rid, 0, _on_depth_data_received)
		else:
			_pending_depth = false

	_check_capture_complete()

func _on_data_received(data: PackedByteArray) -> void:
	if not data.is_empty():
		_cached_image_msg.header.stamp = _current_stamp
		_cached_image_msg.data = data
		_cached_info_msg.header.stamp = _current_stamp
		_camera_pub.publish(_cached_image_msg)
		_camera_info_pub.publish(_cached_info_msg)
	_pending_color = false
	_check_capture_complete()

func _on_depth_data_received(data: PackedByteArray) -> void:
	if not data.is_empty():
		_cached_depth_msg.header.stamp = _current_stamp
		_cached_depth_msg.data = data
		_cached_depth_info_msg.header.stamp = _current_stamp
		_depth_pub.publish(_cached_depth_msg)
		_depth_info_pub.publish(_cached_depth_info_msg)
	_pending_depth = false
	_check_capture_complete()

func _check_capture_complete() -> void:
	if not _pending_color and not _pending_depth:
		is_requesting = false

func _cache_message_templates(camera: Camera3D) -> void:
	_cached_image_msg = RosSensorMsgsImage.new()
	_cached_image_msg.height = resolution.y
	_cached_image_msg.width = resolution.x
	_cached_image_msg.encoding = "rgba8"
	_cached_image_msg.step = resolution.x * 4
	_cached_image_msg.header.frame_id = _resolved_optical_frame
	
	_cached_info_msg = RosSensorMsgsCameraInfo.new()
	_cached_info_msg.header.frame_id = _resolved_optical_frame
	_cached_info_msg.width = resolution.x
	_cached_info_msg.height = resolution.y
	_cached_info_msg.distortion_model = "plumb_bob"
	_cached_info_msg.d = PackedFloat64Array([0.0, 0.0, 0.0, 0.0, 0.0])
	
	var fovy_rad = deg_to_rad(camera.fov)
	var fy = resolution.y / (2.0 * tan(fovy_rad / 2.0))
	var fx = fy 
	var cx = resolution.x / 2.0
	var cy = resolution.y / 2.0
	
	_cached_info_msg.k = PackedFloat64Array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
	_cached_info_msg.r = PackedFloat64Array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
	_cached_info_msg.p = PackedFloat64Array([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])

	if publish_depth:
		# The depth image is rendered from the same viewpoint as the color image (same camera,
		# same render pass), so it is already pixel-aligned and shares its optical frame/intrinsics.
		_cached_depth_msg = RosSensorMsgsImage.new()
		_cached_depth_msg.height = resolution.y
		_cached_depth_msg.width = resolution.x
		_cached_depth_msg.encoding = "32FC1"
		_cached_depth_msg.step = resolution.x * 4
		_cached_depth_msg.header.frame_id = _resolved_optical_frame

		_cached_depth_info_msg = RosSensorMsgsCameraInfo.new()
		_cached_depth_info_msg.header.frame_id = _resolved_optical_frame
		_cached_depth_info_msg.width = resolution.x
		_cached_depth_info_msg.height = resolution.y
		_cached_depth_info_msg.distortion_model = _cached_info_msg.distortion_model
		_cached_depth_info_msg.d = _cached_info_msg.d
		_cached_depth_info_msg.k = _cached_info_msg.k
		_cached_depth_info_msg.r = _cached_info_msg.r
		_cached_depth_info_msg.p = _cached_info_msg.p
