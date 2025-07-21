/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    return (1 - std::fabs(p.x())) * (1 - std::fabs(p.y()));
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float radius_sqr = std::sqrt(sample.x());
    float angle = 2.0f * EIGEN_PI * sample.y();
    return Point2f(radius_sqr * std::cos(angle), radius_sqr * std::sin(angle));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return p.squaredNorm() <= 1.0f ? 1.0f / EIGEN_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float t = 2.0f * sample.x() - 1.0f;
    int sign = t >= 0.0f ? 1 : -1;
    float theta = sign * EIGEN_PI * std::sqrt(std::fabs(t));
    float phi = 2.0f * EIGEN_PI * sample.y();

    float costheta = std::cos(theta);
    float sintheta = std::sin(theta);

    return Vector3f(sintheta * std::cos(phi), sintheta * std::sin(phi), costheta);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    float norm = v.norm();
    float half = 0.01f;
    float radius = 1.0f;
    return (norm >= radius - half && norm <= radius + half) ? 1.0f / (8.0f * EIGEN_PI * half) : 0.0f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() > 0.0f && v.squaredNorm() <= 1.0f ? 1.0f : 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    float costheta = v.dot(Vector3f(0, 0, 1));
    return costheta >= 0.0f ? costheta / EIGEN_PI : 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    auto cos_theta = m.z();
    auto cos_theta_sq = cos_theta * cos_theta;
    auto tan_theta_sq = (1 - cos_theta_sq) / cos_theta_sq;
    auto alpha_sq = alpha * alpha;
    auto beckmann = 2.0f * std::exp(-tan_theta_sq / alpha_sq);
    beckmann /= alpha_sq * cos_theta * cos_theta_sq;
    beckmann /= 2 * EIGEN_PI;
    return beckmann;
}

NORI_NAMESPACE_END
