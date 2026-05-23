#[compute]
#version 450

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(set=0, binding=0) uniform sampler2D depth_buffer;
layout(set=0, binding=1) uniform sampler2D color_buffer;
layout(set=0, binding=2, rgba32f) uniform restrict writeonly image2D face_out;

void main() {
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    ivec2 dest_size = imageSize(face_out);
    
    if (xy.x >= dest_size.x || xy.y >= dest_size.y) {
        return;
    }

    // Normalize coordinates
    vec2 uv = (vec2(xy) + 0.5) / vec2(dest_size);

    // Sample depth (Godot depth is [0, 1])
    float depth = texture(depth_buffer, uv).r;

    // Sample color and compute average intensity
    vec4 color = texture(color_buffer, uv);
    float intensity = (color.r + color.g + color.b) / 3.0;

    // Store in output face texture: R = depth, G = intensity
    imageStore(face_out, xy, vec4(depth, intensity, 0.0, 0.0));
}
