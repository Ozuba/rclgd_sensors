extends Camera3D

## Camera with toggleable freecam mode for prototyping.
class_name Freecam3D

# --- Configuration ---
@export var toggle_key: Key = KEY_TAB
@export var release_key: Key = KEY_ESCAPE
@export var invert_speed_controls: bool = false
@export var mouse_sensitivity: float = 0.002
@export var acceleration: float = 0.1
@export var min_speed: float = 0.1
@export var max_speed: float = 10.0

# --- Internal State ---
@onready var pivot := Node3D.new()
var target_speed := 2.0
var velocity := Vector3.ZERO

func _ready() -> void:
	_setup_nodes.call_deferred()
	_add_keybindings()
	# Initial capture if this camera starts as current
	if current:
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _setup_nodes() -> void:
	# Create a pivot so we can rotate Y (pivot) and X (camera) independently
	# This prevents "gimbal lock" or tilted horizons
	get_parent().add_child.call_deferred(pivot)
	await get_tree().process_frame
	
	pivot.name = "FreecamPivot"
	pivot.global_transform = self.global_transform
	
	self.reparent(pivot)
	self.position = Vector3.ZERO
	self.rotation = Vector3.ZERO

func _process(delta: float) -> void:
	if not current:
		return
		
	var dir = Vector3.ZERO
	if Input.is_action_pressed("__debug_camera_forward"): dir.z -= 1
	if Input.is_action_pressed("__debug_camera_back"):    dir.z += 1
	if Input.is_action_pressed("__debug_camera_left"):    dir.x -= 1
	if Input.is_action_pressed("__debug_camera_right"):   dir.x += 1
	if Input.is_action_pressed("__debug_camera_up"):      dir.y += 1
	if Input.is_action_pressed("__debug_camera_down"):    dir.y -= 1
	
	if dir != Vector3.ZERO:
		dir = dir.normalized()
		# Rotate direction based on pivot's horizontal heading
		dir = dir.rotated(Vector3.UP, pivot.rotation.y)
		# Rotate direction based on camera's vertical pitch
		dir = dir.rotated(pivot.transform.basis.x, rotation.x)
	
	velocity = lerp(velocity, dir * target_speed, acceleration)
	pivot.position += velocity

func _input(event: InputEvent) -> void:
	# 1. Toggle Capture Mode
	if event.is_action_pressed("__debug_camera_release_mouse"):
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_LEFT and current:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

	# 2. Freecam Controls (Only active when mouse is captured)
	if current and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		if event is InputEventMouseMotion:
			pivot.rotate_y(-event.relative.x * mouse_sensitivity)
			self.rotate_x(-event.relative.y * mouse_sensitivity)
			self.rotation.x = clamp(self.rotation.x, -PI/2, PI/2)
		
		# Speed scaling
		if event is InputEventMouseButton and event.pressed:
			var speed_step = 0.5
			if event.button_index == MOUSE_BUTTON_WHEEL_UP:
				target_speed = clamp(target_speed + speed_step, min_speed, max_speed)
			if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
				target_speed = clamp(target_speed - speed_step, min_speed, max_speed)

func _add_keybindings() -> void:
	var actions = InputMap.get_actions()
	var bindings = {
		"__debug_camera_forward": KEY_W,
		"__debug_camera_back": KEY_S,
		"__debug_camera_left": KEY_A,
		"__debug_camera_right": KEY_D,
		"__debug_camera_up": KEY_SPACE,
		"__debug_camera_down": KEY_SHIFT,
		"__debug_camera_release_mouse": KEY_ESCAPE,
		"__debug_camera_toggle": toggle_key
	}
	
	for action in bindings:
		if action not in actions:
			_add_key_input_action(action, bindings[action])

func _add_key_input_action(action_name: String, key: Key) -> void:
	var ev = InputEventKey.new()
	ev.physical_keycode = key
	InputMap.add_action(action_name)
	InputMap.action_add_event(action_name, ev)
