
#include "skinning.hpp"
#ifdef SCENE_SKINNING

#include "skinning_loader.hpp"


using namespace vcl;




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    shader_mesh = shaders["mesh"];
    /*
    keyframes = { { {-1,0,1}     , 0.0f  },
                  { {0,0,1}    , 1.0f  },
                  { {1,0,1}    , 2.0f  },
                  { {1,0,2}    , 2.5f  },
                  { {2,0,2}    , 3.0f  },
                  { {3,0,2}    , 3.5f  },
                  { {2,0,0}  , 3.75f  },
                  { {1.5,0,-1} , 5.0f  },
                  { {0.5,0,-1}   , 6.0f  },
                  { {0,0,-0.5} , 7.0f },
                  { {-1,0,-0.5}, 8.0f },
    };
    */
    keyframes = { { {1,0,-1}    , 0.0f  },
                  { {1,0,-1}    , 0.01f  },
                  { {2,0,-3}    , 1.0f  },
                  { {3,0,-1}    , 2.0f  },
                  { {4,0,-1}    , 3.0f  },
                  { {4,0,-3}    , 4.0f  },
                  { {6,0,-1}    , 5.0f  },
                  { {6,0,-3}    , 6.0f  },
                  { {8,0,-3}    , 7.0f  },
                  { {8,0,-1}    , 8.0f  },
                  { {9,0,-1}    , 9.0f  },
                  { {9,0,-3}    , 10.0f },
                  { {10,0,-2}   , 11.0f },
                  { {11,0,-3}   , 12.0f },
                  { {11,0,-1}   , 13.0f },
                  { {13,0,-3}   , 14.0f },
                  { {14,0,-2.5}   , 15.0f },
                  { {13,0,-2}   , 16.0f },
                  { {14,0,-1.5}   , 17.0f },
                  { {13,0,-1}   , 18.0f },
                  { {15,0,-1}   , 19.0f },
                  { {15,0,-3}   , 20.0f },
                  { {16.25,0,-2}   , 21.0f },
                  { {15,0,-1}   , 22.0f },
                  { {18,0,-1}   , 23.0f },
                  { {17,0,-2}   , 24.0f },
                  { {17.5,0,-3}   , 24.5f },
                  { {18,0,-2.5}   , 25.0f },
                  { {18.5,0,-3}   , 25.5f },
                  { {19,0,-2}   , 26.0f },
                  { {18,0,-1}   , 27.0f },
                  { {19,0,-1}   , 28.0f },
    };

    timer.t_min = keyframes[0].t;
    timer.t_max = keyframes[keyframes.size() - 1].t;
    timer.t = timer.t_min;

    current_pos = keyframes[0].p;

    keyframe_visual = mesh_primitive_sphere();
    keyframe_visual.shader = shaders["mesh"];
    keyframe_visual.uniform.color = { 1,1,1 };
    keyframe_visual.uniform.transform.scaling = 0.05f;
    keyframe_picked = mesh_primitive_sphere();
    keyframe_picked.shader = shaders["mesh"];
    keyframe_picked.uniform.color = { 1,0,0 };
    keyframe_picked.uniform.transform.scaling = 0.055f;


    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    glEnable(GL_POLYGON_OFFSET_FILL);

    // Init gui parameters
    gui_param.display_mesh      = false;
    gui_param.display_wireframe = false;
    gui_param.display_rest_pose = false;
    gui_param.display_skeleton_bones  = true;
    gui_param.display_skeleton_joints = true;
    gui_param.display_texture = true;
    gui_param.display_type = display_cylinder;

    // Sphere used to display joints
    sphere = mesh_primitive_sphere(0.005f);
    sphere.shader = shader_mesh;

    frame = mesh_primitive_frame();
    frame.uniform.transform.scaling = 0.02f;
    frame.shader = shaders["mesh"];

    trajectory = curve_dynamic_drawable(175); // number of steps stored in the trajectory
    trajectory.uniform.color = { 0,0,1 };

    load_character_data(skeleton, skinning, character_visual, timer, shader_mesh);

    animation_timer_max = timer.t_max;
    timer.t_max = keyframes[keyframes.size() - 1].t;
    picked_object = -1;
}

buffer<joint_geometry> interpolate_skeleton_at_time(float time, const buffer< buffer<joint_geometry_time> >& animation)
{
    // Compute skeleton corresponding to a given time from key poses
    // - animation[k] corresponds to the kth animated joint (vector of joint geometry at given time)
    // - animation[k][i] corresponds to the ith pose of the kth joint
    //      - animation[k][i].time         -> corresponding time
    //      - animation[k][i].geometry.p/r -> corresponding position, rotation

    size_t N_joint = animation.size();
    buffer<joint_geometry> skeleton;
    skeleton.resize(N_joint);

    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const buffer<joint_geometry_time>& joint_anim = animation[k_joint];

        // Find the index corresponding to the current time
        size_t k_current = 0;
        assert_vcl_no_msg(joint_anim.size()>k_current+1);
        while( time>joint_anim[k_current+1].time ) {
            ++k_current;
            assert_vcl_no_msg(joint_anim.size()>k_current+1);
        }

        // TO DO ...
        // Compute correct interpolation of joint geometry
        // (the following code corresponds to nearest neighbors, not to a smooth interpolation)
        joint_geometry current_geometry = joint_anim[k_current].geometry;

        const float alpha = (time - joint_anim[k_current].time) / (joint_anim[k_current + 1].time - joint_anim[k_current].time);
        current_geometry.p = (1.0f - alpha) * current_geometry.p + alpha * joint_anim[k_current+1].geometry.p;
        current_geometry.r = slerp(current_geometry.r, joint_anim[k_current + 1].geometry.r, alpha);


        skeleton[k_joint] = current_geometry;


    }

    return skeleton;
}

void compute_skinning(skinning_structure& skinning,
                      const buffer<joint_geometry>& skeleton_current,
                      const buffer<joint_geometry>& skeleton_rest_pose)
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        // TO DO ...
        // Compute skinning deformation
        // Change the following line to compute the deformed position from skinning relation
        vec3 p0 = skinning.rest_pose[k];
        auto influences = skinning.influence[k];
        size_t N_joint = influences.size();
        mat4 sum = mat4::zero();
        for (size_t i = 0; i < N_joint; i++)
        {
            auto joint_rest = skeleton_rest_pose[influences[i].joint];
            auto weight = influences[i].weight;
            auto mat_rota_rest = joint_rest.r.matrix();
            auto pos_rest = joint_rest.p;
            auto joint_cur = skeleton_current[influences[i].joint];
            auto mat_rota_cur = joint_cur.r.matrix();
            auto pos_cur = joint_cur.p;
            mat4 T_cur = mat4(mat_rota_cur, pos_cur);
            mat4 T_rest_in = mat4(transpose(mat_rota_rest), -inverse(mat_rota_rest) * pos_rest);
            sum = sum + weight * T_cur * T_rest_in;
        }
        vec4 tmp = vec4(p0.x, p0.y, p0.z, 1.0);
        tmp = sum * tmp;
        skinning.deformed.position[k] = vec3(tmp.x, tmp.y, tmp.z);
    }
}



// Convert skeleton from local to global coordinates
buffer<joint_geometry> local_to_global(const buffer<joint_geometry>& local, const buffer<joint_connectivity>& connectivity, const vec3 position)
{
    const size_t N = connectivity.size();
    assert(local.size()==connectivity.size());
    std::vector<joint_geometry> global;
    global.resize(N);
    global[0] = local[0];
    // T_global = T_global^parent * T_local (T: 4x4 transformation matrix)
    //   => R_global = R_global^parent * R_local
    //   => P_global = R_global^parent * P_local + P_global^parent
    for(size_t k=1; k<N; ++k)
    {
        const int parent = connectivity[k].parent;
        global[k].r = global[parent].r * local[k].r;
        global[k].p = global[parent].r.apply(local[k].p) + global[parent].p;
    }

    return global;
}


void display_skeleton(const buffer<joint_geometry>& skeleton_geometry,
                      const buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;

        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_im"),scene.camera);
    }
}

void display_joints(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& sphere)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        sphere.uniform.transform.translation = skeleton_geometry[k].p;
        draw(sphere, scene.camera);
    }

}

void display_frames(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& frame)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        frame.uniform.transform.rotation = skeleton_geometry[k].r.matrix();
        frame.uniform.transform.translation = skeleton_geometry[k].p;
        draw(frame, scene.camera);
    }
}

static size_t index_at_value(float t, vcl::buffer<vec3t> const& v)
{
    const size_t N = v.size();
    assert(v.size() >= 2);
    assert(t >= v[0].t);
    assert(t < v[N - 1].t);

    size_t k = 0;
    while (v[k + 1].t < t)
        ++k;
    return k;
}

static vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3)
{
    float mu = 1.5f;
    vec3 di = mu * ((p2 - p0) / (t2 - t0));
    vec3 du = mu * ((p3 - p1) / (t3 - t1));
    float s = (t - t1) / (t2 - t1);
    vec3 p = (2 * pow(s, 3) - 3 * pow(s, 2) + 1) * p1 +
        (pow(s, 3) - 2 * pow(s, 2) + s) * di +
        (-2 * pow(s, 3) + 3 * pow(s, 2)) * p2 +
        (pow(s, 3) - pow(s, 2)) * du;
    return p;
}

static vec3 mult_vec3_mat3(vec3 v, mat3 m)
{
    return vec3(dot(v, m.col(0)), dot(v, m.col(1)), dot(v, m.col(2)));
}

void scene_model::mouse_click(scene_structure& scene, GLFWwindow* window, int, int, int)
{
    // Mouse click is used to select a position of the control polygon
    // ******************************************************************** //

    // Cursor coordinates
    const vec2 cursor = glfw_cursor_coordinates_window(window);

    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left = glfw_mouse_pressed_left(window);
    const bool key_shift = glfw_key_shift_pressed(window);

    // Check if shift key is pressed
    if (mouse_click_left && key_shift)
    {
        // Create the 3D ray passing by the selected point on the screen
        const ray r = picking_ray(scene.camera, cursor);

        // Check if this ray intersects a position (represented by a sphere)
        //  Loop over all positions and get the intersected position (the closest one in case of multiple intersection)
        const size_t N = keyframes.size();
        picked_object = -1;
        float distance_min = 0.0f;
        for (size_t k = 0; k < N; ++k)
        {
            const vec3 c = keyframes[k].p;
            const picking_info info = ray_intersect_sphere(r, c, 0.1f);

            if (info.picking_valid) // the ray intersects a sphere
            {
                const float distance = norm(info.intersection - r.p); // get the closest intersection
                if (picked_object == -1 || distance < distance_min) {
                    distance_min = distance;
                    picked_object = k;
                }
            }
        }
    }

}

void scene_model::mouse_move(scene_structure& scene, GLFWwindow* window)
{

    const bool mouse_click_left = glfw_mouse_pressed_left(window);
    const bool key_shift = glfw_key_shift_pressed(window);
    if (mouse_click_left && key_shift && picked_object != -1)
    {
        // Translate the selected object to the new pointed mouse position within the camera plane
        // ************************************************************************************** //

        // Get vector orthogonal to camera orientation
        const mat4 M = scene.camera.camera_matrix();
        const vec3 n = { M(0,2),M(1,2),M(2,2) };

        // Compute intersection between current ray and the plane orthogonal to the view direction and passing by the selected object
        const vec2 cursor = glfw_cursor_coordinates_window(window);
        const ray r = picking_ray(scene.camera, cursor);
        vec3& p0 = keyframes[picked_object].p;
        const picking_info info = ray_intersect_plane(r, n, p0);

        // translate the position
        p0 = info.intersection;

    }
}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();
    const float t = timer.t;

    if (t < timer.t_min + 0.1f)
        trajectory.clear();

    size_t idx = index_at_value(t, keyframes);

    const size_t N = keyframes.size();

    const float t1 = keyframes[idx].t;
    const float t2 = keyframes[idx + 1].t;

    const vec3& p1 = keyframes[idx].p;
    const vec3& p2 = keyframes[idx + 1].p;

    const float t0 = keyframes[(idx - 1) % N].t;
    const vec3& p0 = keyframes[(idx - 1) % N].p;
    if (idx + 2 >= N)
        idx = idx - 1;
    const float t3 = keyframes[idx + 2].t;
    const vec3& p3 = keyframes[idx + 2].p;

    const vec3 p = cardinal_spline_interpolation(t, t0, t1, t2, t3, p0, p1, p2, p3);
    trajectory.draw(shaders["curve"], scene.camera);
    for (size_t k = 0; k < N; ++k)
    {
        const vec3& p_keyframe = keyframes[k].p;
        keyframe_visual.uniform.transform.translation = p_keyframe;
        draw(keyframe_visual, scene.camera);
    }
    if (picked_object != -1)
    {
        const vec3& p_keyframe = keyframes[picked_object].p;
        keyframe_picked.uniform.transform.translation = p_keyframe;
        draw(keyframe_picked, scene.camera);
    }
    /*
    for (size_t k = 0; k < N - 1; ++k)
    {
        const vec3& pa = keyframes[k].p;
        const vec3& pb = keyframes[k + 1].p;

        segment_drawer.uniform_parameter.p1 = pa;
        segment_drawer.uniform_parameter.p2 = pb;
        segment_drawer.draw(shaders["segment_im"], scene.camera);
    }
    */
    // Store current trajectory of point p
    trajectory.add_point(p);

    vec3 direction = p - current_pos;
    current_pos = p;
    vec3 base = vec3(0, 0, -1);
    vec3 axis = vec3(0, 1, 0);
    float angle = acos(dot(direction, base) / (norm(direction) * norm(base)));
    if (direction.x > 0)
        angle = -angle;
    //mat3 rotation_matrix = rotation_from_axis_angle_mat3(axis, angle);
    quaternion rotation_quat = axis_angle(axis, angle);

    auto skeleton_geometry_local  = interpolate_skeleton_at_time(fmod(t,animation_timer_max), skeleton.anim);


    skeleton_geometry_local[0].r = rotation_quat * skeleton_geometry_local[0].r;
    skeleton_geometry_local[0].p = skeleton_geometry_local[0].p + p;


    const auto skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity, p);
    auto skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity, p);

    
    if(gui_param.display_rest_pose)
        skeleton_current = skeleton_rest_pose;

    compute_skinning(skinning, skeleton_current, skeleton_rest_pose);
    /*
    for (size_t i = 0; i < skinning.deformed.position.size(); i++)
    {
        skinning.deformed.position[i] = mult_vec3_mat3(skinning.deformed.position[i], rotation_matrix);
        skinning.deformed.position[i] = skinning.deformed.position[i] + p;
    }
    */  
    character_visual.update_position(skinning.deformed.position);


    character_visual.update_normal(skinning.deformed.normal);
    /*
    if(gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    if(gui_param.display_skeleton_joints)
        display_joints(skeleton_current, scene, sphere);
    if(gui_param.display_skeleton_frames)
        display_frames(skeleton_current, scene, frame);
        */

    glPolygonOffset( 1.0, 1.0 );
    GLuint const texture_id = (gui_param.display_texture? character_visual.texture_id : scene.texture_white);
    draw(character_visual, scene.camera, character_visual.shader, texture_id);
    
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(character_visual, scene.camera, shaders["wireframe_quads"]);
    }
}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Timer",  &timer.t, timer.t_min, timer.t_max, "%.2f s");
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    /*
    ImGui::Text("Display Mesh:");
    ImGui::Checkbox("Mesh", &gui_param.display_mesh); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe); ImGui::SameLine();
    ImGui::Checkbox("Texture", &gui_param.display_texture);
    
    ImGui::Text("Display Skeleton:");
    ImGui::Checkbox("Bones", &gui_param.display_skeleton_bones);  ImGui::SameLine();
    ImGui::Checkbox("Joints", &gui_param.display_skeleton_joints);  ImGui::SameLine();
    ImGui::Checkbox("Frames", &gui_param.display_skeleton_frames);

    
    ImGui::Text("Shape type:");
    bool click_cylinder = ImGui::RadioButton("Cylinder", &gui_param.display_type, display_cylinder); ImGui::SameLine();
    bool click_bar = ImGui::RadioButton("Bar", &gui_param.display_type, display_bar); ImGui::SameLine();
    bool click_character = ImGui::RadioButton("Character", &gui_param.display_type, display_character);

    if(click_cylinder)  load_cylinder_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_bar)       load_rectangle_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_character) load_character_data(skeleton, skinning, character_visual, timer, shader_mesh);


    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
    */
    // Start and stop animation
    bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
    bool const start = ImGui::Button("Start"); ImGui::SameLine();

    if(stop) timer.stop();
    if(start) timer.start();
}



#endif
