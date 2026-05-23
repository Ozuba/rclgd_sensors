#[compute]
#version 450

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// 6 input face textures (R = depth, G = intensity)
layout(set=0, binding=0, rgba32f) uniform restrict readonly image2D face0;
layout(set=0, binding=1, rgba32f) uniform restrict readonly image2D face1;
layout(set=0, binding=2, rgba32f) uniform restrict readonly image2D face2;
layout(set=0, binding=3, rgba32f) uniform restrict readonly image2D face3;
layout(set=0, binding=4, rgba32f) uniform restrict readonly image2D face4;
layout(set=0, binding=5, rgba32f) uniform restrict readonly image2D face5;

layout(set=0, binding=6, std430) restrict buffer Params {
    vec2 resolution;       // Output resolution (H_RES, V_RES)
    float seed;            // Time-based seed to animate noise
    float noise_std_dev;   // Standard deviation for Gaussian noise
    float vertical_fov_min;// Rad
    float vertical_fov_max;// Rad
    float min_range;       // Meters
    float max_range;       // Meters
    mat4 inv_proj_matrices[6];
    mat4 cam_transforms[6];
    mat4 inv_cam_transforms[6];
} params;

// Output texture containing ROS point cloud data (RGBA32F: RGB = XYZ in ROS, A = intensity)
layout(set=0, binding=7, rgba32f) uniform restrict writeonly image2D lidar_out;

const float PI = 3.14159265359;

// Generate uniform random numbers
float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233) + params.seed)) * 43758.5453);
}

// Box-Muller Transform for Gaussian noise
float get_gaussian(vec2 co) {
    float u1 = max(rand(co), 1e-6); 
    float u2 = rand(co + 1.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2) * params.noise_std_dev;
}

void main() {
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    ivec2 dest_size = ivec2(params.resolution);

    if (xy.x >= dest_size.x || xy.y >= dest_size.y) {
        return;
    }

    // 1. Calculate the spherical ray direction in Lidar local space
    // Normalize coordinates
    float u = (float(xy.x) + 0.5) / params.resolution.x;
    float w = (float(xy.y) + 0.5) / params.resolution.y;

    float theta = u * 2.0 * PI - PI; // Yaw: [-PI, PI]
    float phi = params.vertical_fov_min + w * (params.vertical_fov_max - params.vertical_fov_min); // Pitch

    vec3 d_local;
    d_local.x = sin(theta) * cos(phi);
    d_local.y = sin(phi);
    d_local.z = -cos(theta) * cos(phi); // Forward is -Z in Godot

    // 2. Find which camera face this direction vector projects onto
    int best_cam = -1;
    float max_z_front = -1e9;
    float best_x_ndc = 0.0;
    float best_y_ndc = 0.0;

    for (int i = 0; i < 6; ++i) {
        vec3 d_cam = (params.inv_cam_transforms[i] * vec4(d_local, 0.0)).xyz;
        if (d_cam.z < 0.0) { // Camera looks down -Z, so front is negative Z
            float front_z = -d_cam.z;
            float x_ndc = d_cam.x / front_z;
            float y_ndc = d_cam.y / front_z;
            
            // Allow a small epsilon for floating-point margins
            if (abs(x_ndc) <= 1.001 && abs(y_ndc) <= 1.001) {
                if (front_z > max_z_front) {
                    max_z_front = front_z;
                    best_cam = i;
                    best_x_ndc = x_ndc;
                    best_y_ndc = y_ndc;
                }
            }
        }
    }

    // If no camera face was found (edge case), output an empty point
    if (best_cam == -1) {
        imageStore(lidar_out, xy, vec4(0.0, 0.0, 0.0, 0.0));
        return;
    }

    // 3. Load depth and intensity from the selected face texture
    // Texture coordinates have V=0 at the top (which corresponds to positive y_ndc)
    vec2 lookup_uv;
    lookup_uv.x = best_x_ndc * 0.5 + 0.5;
    lookup_uv.y = 0.5 - best_y_ndc * 0.5; // Vulkan Y-flip

    ivec2 face_size = imageSize(face0); // Assuming all faces are same size
    ivec2 face_xy = ivec2(lookup_uv * vec2(face_size));
    face_xy = clamp(face_xy, ivec2(0), face_size - 1);

    vec4 val = vec4(0.0);
    switch (best_cam) {
        case 0: val = imageLoad(face0, face_xy); break;
        case 1: val = imageLoad(face1, face_xy); break;
        case 2: val = imageLoad(face2, face_xy); break;
        case 3: val = imageLoad(face3, face_xy); break;
        case 4: val = imageLoad(face4, face_xy); break;
        case 5: val = imageLoad(face5, face_xy); break;
    }

    float depth_raw = val.r;
    float intensity = val.g;

    // In Godot 4, Vulkan uses reversed-Z depth buffer (1.0 = near plane, 0.0 = far plane).
    // A depth of 0.0 (or very close) means nothing was hit (sky/far plane).
    if (depth_raw <= 0.0001) {
        imageStore(lidar_out, xy, vec4(0.0, 0.0, 0.0, 0.0));
        return;
    }

    // 4. Reconstruct the view-space position in the face camera's space
    vec4 ndc = vec4(best_x_ndc, best_y_ndc, depth_raw, 1.0);
    vec4 view_pos = params.inv_proj_matrices[best_cam] * ndc;
    view_pos /= view_pos.w;

    // 5. Transform back to Lidar local space
    vec4 local_point = params.cam_transforms[best_cam] * vec4(view_pos.xyz, 1.0);

    // Calculate actual distance/range
    float dist = length(local_point.xyz);

    // Apply range limits
    if (dist < params.min_range || dist > params.max_range) {
        imageStore(lidar_out, xy, vec4(0.0, 0.0, 0.0, 0.0));
        return;
    }

    // 6. Add Gaussian noise along the ray
    float noise = get_gaussian(vec2(xy));
    float noisy_dist = max(dist + noise, params.min_range);
    vec3 local_noisy_point = d_local * noisy_dist;

    // 7. Convert to ROS frame:
    // Godot local -> ROS frame
    // ROS X = Forward = -Godot Z
    // ROS Y = Left = -Godot X
    // ROS Z = Up = Godot Y
    vec3 ros_point;
    ros_point.x = -local_noisy_point.z;
    ros_point.y = -local_noisy_point.x;
    ros_point.z = local_noisy_point.y;

    imageStore(lidar_out, xy, vec4(ros_point, intensity));
}
