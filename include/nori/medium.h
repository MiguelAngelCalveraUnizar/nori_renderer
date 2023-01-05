/*
*/

#pragma once

#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/dpdf.h>

#ifndef n_UINT
#define n_UINT uint32_t
#endif

NORI_NAMESPACE_BEGIN

///**
// * \brief Intersection data structure
// *
// * This data structure records local information about a ray-triangle intersection.
// * This includes the position, traveled ray distance, uv coordinates, as well
// * as well as two local coordinate frames (one that corresponds to the true
// * geometry, and one that is used for shading computations).
// */

struct MediumIntersection {
    /// Position for the origin
    Point3f o;
    /// Position for the end (where it hitted an intersection)
    Point3f xz;
    /// Position for the point in the middle of the Medium
    Point3f xt;
    // Probability of point xt sampled
    float pdf_xt;

    /// Local Frame
    Frame shFrame;

    /// Pointer to the associated mesh
    const Medium* medium;

    /// Create an uninitialized intersection record
    MediumIntersection() : medium(nullptr) { };

    /// Transform a direction vector into the local shading frame
    Vector3f toLocal(const Vector3f& d) const {
        return shFrame.toLocal(d);
    }

    /// Transform a direction vector from local to world coordinates
    Vector3f toWorld(const Vector3f& d) const {
        return shFrame.toWorld(d);
    }

    /// Return a human-readable summary of the intersection record
    std::string toString() const;
};

/*
 * \brief Medium
 *
 * This class stores a medium object and provides numerous functions
 * for querying it. Subclasses of \c Mediums implement
 * the specifics of how to create its contents (e.g. by loading from an
 * external file)
 */
class Medium : public NoriObject {
public:
    /// Release all memory
    virtual ~Medium();

    /// Initialize internal data structures (called once by the XML parser)
    virtual void activate();

    //// Return an axis-aligned bounding box of the entire mesh
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    const PhaseFunction *getPhaseFunction() const{ return m_pf; }

    // WE CANNOT HAVE ANY REFERENCE TO MEDIUMINTERSECTION YET, THAT WILL HAVE TO BE DONE IN HOMOGENEUS AND HETEROGENEUS.
    const bool sampleBetween(Point3f x, Point3f xz, float rnd, MediumIntersection &medIts) const;

    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child, const std::string& name = "none");

    /// Return the name of this mesh
    const std::string &getName() const { return m_name; }

    /// Return a human-readable summary of this instance
    std::string toString() const;

    /// For homogeneus and heterogeneus media to define correctly
    const Color3f Transmittance(Point3f x, Point3f xz) const { return Color3f(1); }

    const float getScatteringCoeficient() const { return 1; }

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMedium; }

protected:
    /// Create an empty mesh
    Medium();

protected:
    std::string m_name;                  ///< Identifying name
    BoundingBox3f m_bbox;                ///< Bounding box of the mesh
    PhaseFunction* m_pf = nullptr;

};


NORI_NAMESPACE_END
