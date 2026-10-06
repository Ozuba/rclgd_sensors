extends CanvasLayer
## Frame rate overlay for the demo: FPS, frame time and physics tick rate, top-left corner.

const REFRESH_INTERVAL: float = 0.25 # s

var _label := Label.new()
var _since_refresh: float = 0.0

func _ready() -> void:
	layer = 100 # Above any other UI
	_label.position = Vector2(8, 8)
	_label.add_theme_font_size_override("font_size", 14)
	_label.add_theme_color_override("font_outline_color", Color.BLACK)
	_label.add_theme_constant_override("outline_size", 4)
	add_child(_label)

func _process(delta: float) -> void:
	_since_refresh += delta
	if _since_refresh < REFRESH_INTERVAL:
		return
	_since_refresh = 0.0
	var fps := Engine.get_frames_per_second()
	_label.text = "%d FPS  (%.1f ms)\nPhysics %d Hz" % [
		fps, 1000.0 / maxf(fps, 1.0), Engine.physics_ticks_per_second]
