/*
  /*
    This file is part of Nori, a simple educational ray tracer
    Copyright (c) 2015 by Wenzel Jakob
    v1 - Dec 2020
    Copyright (c) 2020 by Adrian Jarabo
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


#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

Mesh::~Mesh() {
    m_pdf.clear();
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }

    m_pdf.reserve(m_F.cols());
    // We put a probability that is the surface area for each triangle:
    for (n_UINT i = 0;i < m_F.cols();++i) {
        m_pdf.append(surfaceArea(i));
    }
    
    // Now we normalize the pdf, so it divides everything by the total area (now the sum is 1):
    m_pdf.normalize();
}

float Mesh::surfaceArea(n_UINT index) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(n_UINT index, const Ray3f &ray, float &u, float &v, float &t) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(n_UINT index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(n_UINT index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

/**
 * \brief Uniformly sample a position on the mesh with
 * respect to surface area. Returns both position and normal
 */
void Mesh::samplePosition(const Point2f& sample, Point3f& p, Normal3f& n, Point2f& uv) const
{
    // Get things from the mesh:
    const MatrixXf& V = getVertexPositions();
    const MatrixXf& N = getVertexNormals();
    const MatrixXu& F = getIndices();
    const MatrixXf& UV = getVertexTexCoords();

    // Sample a triangle from the mesh at random
    Point2f sampledUV = Warp::squareToUniformTriangle(sample);

    // Get point in barycentric coordinates
    Vector3f bary = Vector3f(sampledUV[0], sampledUV[1], 1 - sampledUV[1] - sampledUV[0]);
    
    // Resample
    float third_sample = sample[0];
    m_pdf.sampleReuse(third_sample); // Allow to re-use the sample

    //Use the sample to get a triangle by it's area (m_pdf is a discrete pdf defined by areaTriangle/totalArea)
    size_t inxT = m_pdf.sample(third_sample);

    // Get sampled triangle points p0, p1 and p2
    n_UINT idx0 = F(0, inxT), idx1 = F(1, inxT), idx2 = F(2, inxT);
    Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);
    
    // Get coordinates of point by interpolation:
    p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;
    // Now we get the normal
    if (N.size() > 0) {
        n = (bary.x() * N.col(idx0) +
            bary.y() * N.col(idx1) +
            bary.z() * N.col(idx2)).normalized();
    }else {
        n = (p1 - p0).cross(p2 - p0).normalized();
    }

    // Finally get correct uv coordinates
    if (UV.size() > 0)
        uv = bary.x() * UV.col(idx0) +
        bary.y() * UV.col(idx1) +
        bary.z() * UV.col(idx2);

}

/// Returns the PDF of sampling a point on the mesh
float Mesh::pdf(const Point3f &p) const
{
    float prob = 0;
    if (m_pdf.isNormalized()) {
        prob = m_pdf.getNormalization();
    }
	return prob;
}


void Mesh::addChild(NoriObject *obj, const std::string& name) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Mesh: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = emitter;
            }
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END