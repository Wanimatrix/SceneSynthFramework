#pragma once

#include "../types.h"
#include <bitset>
#include <string>

enum TransformationType 
{
    TRANSLATION, SCALING, ROTATION
};

class Transformation {
public:

    Transformation(Eigen::Matrix4d transformationMatrix, TransformationType type) : m_matrix(transformationMatrix), m_type(type)
    {}
    virtual ~Transformation() {}

    virtual bool isTranslation() {return m_type == TRANSLATION; }
    virtual bool isScaling() {return m_type == SCALING; }
    virtual bool isRotation() {return m_type == ROTATION; }
    virtual Eigen::Matrix4d getInverseMatrix();
    virtual Eigen::Matrix4d getMatrix();

    static Eigen::Matrix4d getTransformationMatrix(std::stack<Transformation> transformationStack);
    static std::vector<double> getTranslation(std::stack<Transformation> transformationStack);
    static std::stack<Transformation> emptyTransformationStack();

private:
    Eigen::Matrix4d m_matrix;
    TransformationType m_type;
};

