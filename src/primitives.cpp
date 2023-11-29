#include "primitives.h"

#include "gl_debug.h"

common::Mesh::Mesh(float *vertices, size_t vertices_size, u_int32_t *indices, size_t indices_size, u_int32_t indices_count)
        : m_vbo(0), m_ebo(0), m_vao(0), m_count(indices_count) {
    GLCall( glGenVertexArrays(1, &m_vao) );
    GLCall( glGenBuffers(1, &m_vbo) );
    GLCall( glGenBuffers(1, &m_ebo) );

    GLCall( glBindVertexArray(m_vao) );

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_vbo) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, vertices_size, vertices, GL_STATIC_DRAW) );

    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo) );
    GLCall( glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices, GL_STATIC_DRAW) );

    GLCall( glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0) );
    GLCall( glEnableVertexAttribArray(0) );

    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    GLCall( glBindVertexArray(0) );
}

common::Mesh::Mesh()
        : m_vbo(0), m_ebo(0), m_vao(0), m_count(0) {
    GLCall( glGenVertexArrays(1, &m_vao) );
    GLCall( glGenBuffers(1, &m_vbo) );
    GLCall( glGenBuffers(1, &m_ebo) );
}

void common::Mesh::bind() const {
    GLCall( glBindVertexArray(m_vao) );
    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo) );
}

void common::Mesh::unbind() const {
    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    GLCall( glBindVertexArray(0) );
}

common::Box::Box() {

    float vertices[] {
            // front
            -0.5, -0.5,  0.5,
            0.5, -0.5,  0.5,
            0.5,  0.5,  0.5,
            -0.5,  0.5,  0.5,
            // back
            -0.5, -0.5, -0.5,
            0.5, -0.5, -0.5,
            0.5,  0.5, -0.5,
            -0.5,  0.5, -0.5,
    };

    uint32_t indices[] {
            // front
            0, 1, 2,
            2, 3, 0,
            // right
            1, 5, 6,
            6, 2, 1,
            // back
            7, 6, 5,
            5, 4, 7,
            // left
            4, 0, 3,
            3, 7, 4,
            // bottom
            4, 5, 1,
            1, 0, 4,
            // top
            3, 2, 6,
            6, 7, 3
    };

    m_mesh = Mesh(vertices, sizeof(vertices), indices, sizeof(indices), 36);
}

void common::Box::draw(const ShaderProgram &program) {
    program.bind();
    m_mesh.bind();
    GLCall( glDrawElements(GL_TRIANGLES, m_mesh.get_count(), GL_UNSIGNED_INT, nullptr) );
}
