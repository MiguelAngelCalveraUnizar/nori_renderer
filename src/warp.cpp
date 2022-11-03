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
    float r, theta, xOff, yOff, pi_4;
    pi_4 = (float)M_PI_2 / 2;
    xOff = 2 * sample[0] - 1;
    yOff = 2 * sample[1] - 1;

    if (xOff == 0 && yOff == 0){
        return Point2f(0, 0);
    }

    if (std::abs(xOff) > std::abs(yOff)) {
        r = xOff;
        theta = pi_4 * (yOff / xOff);
    }else {
        r = yOff;
        theta = (float)M_PI_2 - pi_4 * (xOff / yOff);
    }
    return Point2f(r * std::cos(theta), r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (std::sqrt(p[0] * p[0] + p[1] * p[1]) < 1) ? INV_PI : 0.0f;
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
    float x, y, z, phi, logSample, cos_theta, sin_theta, tan2theta;

    phi = 2 * M_PI * sample[0];
    logSample = std::log(std::max((float)0, 1-sample[1]));
    // logSample can be a number from -inf to 0.
    tan2theta = -alpha * alpha * logSample;
    // tan2theta can be a number from inf to 0.
    cos_theta = (float)1 / std::sqrt(1 + tan2theta);
    // cos_theta can be a number from 0 to 1.
    sin_theta = std::sqrt(std::max((float)0, (float)1. - cos_theta * cos_theta));

    // Now from polar to cartesian
    x = sin_theta * std::cos(phi);
    y = sin_theta * std::sin(phi);
    z = cos_theta;

    return Vector3f(x,y,z);
}

float Warp::squareToBeckmannPdf(const Vector3f &v, float alpha) {
    float tan2theta, cos_theta, alpha2, prob, denom;
    alpha2 = alpha * alpha;
    // From z we have cos theta and tan
    cos_theta = v[2];

    if (cos_theta == 0) {
        prob = 0;
    }else {
        tan2theta = ((float)1. / (cos_theta * cos_theta))-1;
        denom = M_PI * alpha2 * cos_theta * cos_theta * cos_theta;
        prob = std::exp(-tan2theta / alpha2) / denom;
    }

    return (v[2] >= 0 && (std::abs(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1) < 0.0001)) ? prob : 0.0f;
}

NORI_NAMESPACE_END
