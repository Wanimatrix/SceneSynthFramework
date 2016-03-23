#include "Transformation.h"

Eigen::Matrix4d Transformation::getTransformationMatrix(std::stack<Transformation> transformationStack)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    while(!transformationStack.empty())
    {
        result = result*transformationStack.top().getMatrix();
        transformationStack.pop();
    }

    return result;
}

Eigen::Matrix4d Transformation::getInverseMatrix()
{
    Eigen::Matrix4d result(m_matrix);
    switch(m_type)
    {
        case TransformationType::TRANSLATION:
            result(0,3) *= -1;
            result(1,3) *= -1;
            result(2,3) *= -1;
            break;

        case TransformationType::SCALING:
            result(1,1) = 1/result(1,1);
            result(2,2) = 1/result(2,2);
            result(3,3) = 1/result(3,3);
            break;

        case TransformationType::ROTATION:
            result = result.transpose();
            break;
    }
    return result;
}

Eigen::Matrix4d Transformation::getMatrix() 
{
    return m_matrix;
}

std::stack<Transformation> Transformation::emptyTransformationStack() 
{
    std::stack<Transformation> emptyStack;

    Transformation emptyTransl(Eigen::Matrix4d::Identity(), TransformationType::TRANSLATION);
    emptyStack.push(emptyTransl);

    return emptyStack;
}

std::vector<double> Transformation::getTranslation(std::stack<Transformation> transformationStack)
{
    Eigen::Matrix4d transformMat = Transformation::getTransformationMatrix(transformationStack);
    std::vector<double> result;
    result.push_back(transformMat(0,3));
    result.push_back(transformMat(1,3));
    result.push_back(transformMat(2,3));
    //result[5] = {transformMat(0,3),transformMat(1,3),transformMat(2,3)};

    return result;
}
