#pragma once
#include <vector>

#include <glm/glm.hpp>

#include "Model.h"

namespace android_slam
{

    class Plane2D : public Model
    {
    public:
        struct Vertex
        {
            glm::vec2 position;
            glm::vec2 tex_coords;

            static void resolve()
            {
                glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*)(offsetof(Vertex, position)));
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*)(offsetof(Vertex, tex_coords)));
                glEnableVertexAttribArray(1);
            }
        };

        static const std::vector<Vertex> k_vertices;
        static const std::vector<uint32_t> k_indices;

    public:
        Plane2D() noexcept
            : Model(k_vertices, k_indices)
        {

        }
    };

}