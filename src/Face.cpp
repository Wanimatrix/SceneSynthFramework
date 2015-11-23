#include "Face.h"

Face::Face(const aiFace &t_face) {
    for (int i = 0; i < t_face.mNumIndices; ++i)
    {
        m_vertIndices.push_back(t_face.mIndices[i])
    }
}

Face::~Face() {}