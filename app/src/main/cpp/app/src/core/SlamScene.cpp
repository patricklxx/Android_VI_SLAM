#include "core/SlamScene.h"

#include "core/App.h"

#include "render/ImageTexture.h"
#include "render/Shader.h"
#include "render/Plane2D.h"

#include "utils/Log.h"
#include "utils/AssetManager.h"
#include "utils/ImageLoader.h"


namespace android_slam
{

    void SlamScene::init()
    {
        Image first_image;
        if(!m_from_datasets) {
            // Camera image converter.
            m_image_pool = std::make_unique<ImagePool>(
                    k_sensor_camera_width, k_sensor_camera_height, "shader/yuv2rgb.vert",
                    "shader/yuv2rgb.frag"
            );
            first_image = m_image_pool->getImage();

            //IMU data converter
            m_imu_pool = std::make_unique<SensorIMU>();
        }
        else{
            DatasetImage = ImageLoader::loadDatasetImage("f304/dark_clahe");
            first_image = DatasetImage[0];
            //DatasetImu = ImageLoader::loadDatasetImu("data10/imu.txt");
        }

        m_slam_renderer = std::make_unique<SlamRenderer>(
        k_fps_camera_fov, m_app_ref.getWindow().getAspectRatio(), k_fps_camera_z_min, k_fps_camera_z_max
        );

        // Create slam kernel.
        {
            DEBUG_INFO("[Android Slam App Info] Starts to create slam kernel.");

            AAsset* asset = AAssetManager_open(AssetManager::get(), "vocabulary/ORBVoc.txt", AASSET_MODE_BUFFER);
            assert(asset && "[Android Slam App Info] Failed to open ORBVoc.txt.");

            size_t size = AAsset_getLength(asset);
            auto buffer = (const char*) AAsset_getBuffer(asset);
            std::string voc_buffer(buffer, buffer + size);
            AAsset_close(asset);

            m_slam_kernel = std::make_unique<SlamKernel>(
            k_sensor_camera_width, k_sensor_camera_height, std::move(voc_buffer), first_image.time_stamp
            );

            DEBUG_INFO("[Android Slam App Info] Creates slam kernel successfully.");
        }

        m_slam_has_new_data = false; // Whether the main thread generates new image for slam thread.
        m_is_running_slam = true;     // Whether the slam scene is running, also indicates the slam thread is running.

        // Create slam thread.
        m_slam_thread = std::make_unique<std::thread>(
        [this]()
        {
            std::vector<Image> images;
            std::vector<ImuPoint> imu_points;
            while (m_is_running_slam)
            {
                if (m_slam_has_new_data)
                {
                    // Acquire new images.
                    {
                        std::unique_lock<std::mutex> lock(m_image_mutex);
                        images = std::move(m_images);
                        imu_points = std::move(m_imu_points);
                    }

                    // Call slam tracking function.
                    //auto res = m_slam_kernel->handleData(images, imu_points);
                    auto res = m_slam_kernel->handleData(images, {});
                    m_slam_has_new_data = false; // This image is processed and this thread needs new image.

                    // Synchronize tracking result to main thread, move the data because this thread doesn't need it.
                    {
                        std::unique_lock<std::mutex> lock(m_tracking_res_mutex);
                        m_tracking_result = std::move(res);
                    }

                }
            }
        }
        );
        // Create communication thread.
        if(m_open_comm) {
            m_comm_thread = std::make_unique<std::thread>(
                    [this]() {
                        while (m_is_running_slam) {
                            TrackingResult tracking_res_comm;
                            if (m_comm_has_new_data) {
                                {
                                    std::unique_lock<std::mutex> lock(m_comm_mutex);
                                    tracking_res_comm = m_comm_result;
                                }
                                DEBUG_INFO("[Android Slam App Info] Before communication.");
                                if (tracking_res_comm.tracking_status == "OK") {
                                    /*
                                    comm.Run(tracking_res_comm.trajectory.back().x,
                                             tracking_res_comm.trajectory.back().y,
                                             tracking_res_comm.trajectory.back().z);
                                    */
                                    comm.Run(tracking_res_comm);
                                }
                                DEBUG_INFO("[Android Slam App Info] After communication.");
                                m_comm_has_new_data = false;
                            }

                        }
                    }
            );
        }
    }

    void SlamScene::exit()
    {
        m_is_running_slam = false;
        m_need_update_image = false;
        m_slam_thread->join();
        m_slam_thread.reset(nullptr);

        m_slam_kernel.reset(nullptr);
        m_slam_renderer.reset(nullptr);
        m_image_pool.reset(nullptr);
    }

    void SlamScene::update(float dt)
    {
        // Update image data if not paused and slam thread needs new image. Render data will be set at the same time.
        if (m_need_update_image && !m_slam_has_new_data)
        {
            // Slam handling.
            std::vector<Image> images;
            std::vector<ImuPoint> imus;
            if(!m_from_datasets) {
                images.push_back(m_image_pool->getImage());
                {
                    std::unique_lock<std::mutex> lock(m_image_mutex);
                    m_images = images;
                    m_imu_points = m_imu_pool->getImuData();
                }
                m_slam_has_new_data = true;
            }
            else{
                {
                    images.push_back(DatasetImage[m_datasets_num]);
                    //imus = ImageLoader::getImuDataset(DatasetImu,
                    //                                  DatasetImage[m_datasets_num].time_stamp,
                    //                                  DatasetImage[m_datasets_num-1].time_stamp);
                    m_datasets_num++;
                    usleep(20000);
                    std::unique_lock<std::mutex> lock(m_image_mutex);
                    m_images = images;
                    //m_imu_points = imus;
                }
                m_slam_has_new_data = true;
            }
            TrackingResult tracking_res;
            {
                std::unique_lock<std::mutex> lock(m_tracking_res_mutex);
                tracking_res = std::move(m_tracking_result);
            }
            DEBUG_INFO("[Android Slam App Info] Current tracking state: %s", tracking_res.tracking_status.c_str());

            // Set slam data.
            m_slam_renderer->setImage(k_sensor_camera_width, k_sensor_camera_height, images[0]);
            m_slam_renderer->setData(tracking_res);

            m_app_ref.m_last_process_delta_time = tracking_res.processing_delta_time;

            //set comm data
            if(!m_comm_has_new_data)
            {
                {
                    std::unique_lock<std::mutex> lock(m_comm_mutex);
                    m_comm_result = std::move(tracking_res);
                }
            }
            m_comm_has_new_data = true;
        }

        // Draw trajectory.
        m_slam_renderer->clearColor();

        int32_t screen_width = m_app_ref.getWindow().getWidth();
        int32_t screen_height = m_app_ref.getWindow().getHeight();

        m_slam_renderer->drawMapPoints(0, 0, screen_width, screen_height);
        m_slam_renderer->drawKeyFrames(0, 0, screen_width, screen_height);

        int32_t img_width = screen_height * 4 / 7;
        int32_t img_height = screen_height * 3 / 7;
        m_slam_renderer->drawImage(0, 0, img_width, img_height); // Aspect ratio: 4 : 3

        m_slam_renderer->drawTotalTrajectory(0, img_height, img_width, screen_height - img_height); // Aspect ratio: 1 : 1
    }

    void SlamScene::drawGui(float dt)
    {
        if (ImGui::TreeNode(u8"SLAM控制"))
        {
            //auto pos = m_slam_renderer->temp_last_kf_position;
            //ImGui::Text("Position: (%f, %f, %f)", pos.x, pos.y, pos.z);

            if (ImGui::Button(m_need_update_image ? u8"暂停" : u8"继续"))
            {
                m_need_update_image = !m_need_update_image;
            }

            //if (ImGui::Button(u8"重置"))
            //{
            //    m_slam_kernel->reset();
            //}

            if (ImGui::Button(u8"退出"))
            {
                m_app_ref.setActiveScene("Init");
            }

            ImGui::TreePop();
        }

        if (ImGui::TreeNode(u8"绘制选项"))
        {
            if (ImGui::TreeNode(u8"地图点"))
            {
                if (ImGui::Button(m_slam_renderer->m_show_mappoints ? u8"隐藏" : u8"显示"))
                {
                    m_slam_renderer->m_show_mappoints = !m_slam_renderer->m_show_mappoints;
                }

                ImGui::SliderFloat(u8"地图点大小", &m_slam_renderer->m_point_size, 1.0f, 5.0f);

                ImGui::ColorEdit3(u8"地图点颜色", reinterpret_cast<float*>(&m_slam_renderer->m_mp_color));

                ImGui::TreePop();
            }

            if (ImGui::TreeNode(u8"相机轨迹"))
            {
                if (ImGui::Button(m_slam_renderer->m_show_keyframes ? u8"隐藏" : u8"显示"))
                {
                    m_slam_renderer->m_show_keyframes = !m_slam_renderer->m_show_keyframes;
                }

                ImGui::SliderFloat(u8"轨迹线粗细", &m_slam_renderer->m_line_width, 1.0f, 5.0f);

                ImGui::ColorEdit3(u8"轨迹线颜色", reinterpret_cast<float*>(&m_slam_renderer->m_kf_color));

                ImGui::TreePop();
            }

            if (ImGui::TreeNode(u8"其他"))
            {
                if (ImGui::Button(m_slam_renderer->m_show_image ? u8"隐藏相机图像" : u8"显示相机图像"))
                {
                    m_slam_renderer->m_show_image = !m_slam_renderer->m_show_image;
                }

                if (ImGui::Button(m_slam_renderer->m_show_total_trajectory ? u8"隐藏全局轨迹" : u8"显示全局轨迹"))
                {
                    m_slam_renderer->m_show_total_trajectory = !m_slam_renderer->m_show_total_trajectory;
                }

                ImGui::ColorEdit3(u8"背景颜色", reinterpret_cast<float*>(&m_slam_renderer->m_clear_color));

                ImGui::TreePop();
            }

            ImGui::TreePop();
        }
    }
}