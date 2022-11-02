/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
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
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    float spx = std::sqrt(sample[0]);
    float py = sample[1];
    return Point2f(
        1 - spx,
        py * spx
    );
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    return ((p.array() >= 0).all() && (p.array() <= 1).all() && p[0]+p[1] < 1) ? 2.0f : 0.0f;
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float z = 1 - 2 * sample[0];
    float sz = std::sqrt(1 - z * z);
    return Vector3f(
        std::cos(2 * M_PI * sample[1]) * sz,
        std::sin(2 * M_PI * sample[1]) * sz,
        z
    );
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return (std::abs(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1) < 0.0001) ? INV_FOURPI : 0.0f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = sample[0];
    float r = std::sqrt(std::max((float)0, (float)1. - z * z));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return (v[2] >= 0 && (std::abs(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1) < 0.0001)) ? INV_TWOPI: 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f& sample) {
    float x, y, z;
    float theta = 2 * M_PI * sample[0];
    x = std::cos(theta) * std::sqrt(std::max((float)0, (float)sample[1]));
    y = std::sin(theta) * std::sqrt(std::max((float)0, (float)sample[1]));
    z = std::sqrt(std::max((float)0, (float)1 - sample[1]));
    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f& v) {
    float prob = v[2] * INV_PI;
    return (v[2] >= 0 && (std::abs(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1) < 0.0001)) ? prob : 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float logSample, phi, tan2theta, cos2theta, costheta, sintheta, x, y, z;
    logSample = std::log(1 - sample[0]);
    if (std::isinf(logSample)) logSample = 0;

    tan2theta = -alpha * alpha * logSample;
    phi = 2 * M_PI * sample[1];
    
    // Now we calculate cos tita and sen tita.
    //cos2theta = 1 / (1 + tan2theta);
    //costheta = std::sqrt(cos2theta);
    //sintheta = std::sqrt(std::max((float)0, 1 - cos2theta));

    costheta = 1 / std::sqrt(1 + tan2theta);
    sintheta = std::sqrt(std::max((float)0, 1 - costheta * costheta));

    // Now from spherical to cartesian coordinates.
    x = sintheta * std::cos(phi);
    y = sintheta * std::sin(phi);
    z = costheta;

    return Vector3f(x,y,z);
}

float Warp::squareToBeckmannPdf(const Vector3f &v, float alpha) {
    float theta, tan_theta, cos_theta, alpha2, prob;
    cos_theta = v[2];
    theta = std::acos(cos_theta);
    tan_theta = std::tan(theta);
    alpha2 = alpha * alpha;
    prob = 2*std::sin(theta) * std::expf(-tan_theta * tan_theta / alpha2) / (alpha2 * cos_theta * cos_theta * cos_theta);
    if (std::isnan(prob)) prob = 0;
    return (v[2] >= 0 && (std::abs(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1) < 0.0001)) ? prob : 0.0f;
}

NORI_NAMESPACE_END
