/*
*/

#pragma once

#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/dpdf.h>
#include <nori/volume.h>

#ifndef n_UINT
#define n_UINT uint32_t
#endif

NORI_NAMESPACE_BEGIN

///**
// * \brief MediumIntersection data structure
// *
// * This data structure records information about a ray-triangle intersection and the medium it passes through.
// * This includes the origin of the ray, the point where the ray intersects with the scene, 
// * as well as the points where it enters or goes out of the medium and a sample done in the middle of it for the inscattering.
// * as well as the local coordinate frames for the point in the medium (that depends on orientation of the particle)
// */

struct MediumIntersection {
    /// Position for the origin of the ray
    Point3f o;
    /// Position of the point that intersects with something in the scene
    Point3f p;
    // Position of the beggining of the medium
    Point3f x;
    /// Position for the end of the medium
    Point3f xz;
    /// Position for the point in the middle of the Medium (for inscattering)
    Point3f xt;
    // Probability of a point xt sampled
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
    //const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    // This cannot be const cause m_accel_medium->addMesh() need as a parameter a non const Mesh*
    Mesh* getBoundingBoxAsMesh() const { return m_mesh; }

    const PhaseFunction *getPhaseFunction() const{ return m_pf; }

    virtual void sampleBetween(float rnd, MediumIntersection &medIts) const;

    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child, const std::string& name = "none");

    /// Return the name of this mesh
    const std::string &getName() const { return m_name; }

    /// Return a human-readable summary of this instance
    std::string toString() const;

    /// For homogeneus and heterogeneus media to define correctly
    virtual const float Transmittance(Point3f x, Point3f xz) const { return 1.0f; }

    // Check that this isn't a problem:
    virtual const float getScatteringCoeficient() const { return 0.0f; }

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMedium; }
    
    //TODO CHANGE
    VolumeDataSource* m_vol_density = nullptr;
    VolumeDataSource* m_vol_orientation = nullptr;
protected:
    /// Create an empty mesh
    Medium();

protected:
    std::string m_name;                  ///< Identifying name
    //BoundingBox3f m_bbox;                ///< Bounding box of the mesh
    PhaseFunction* m_pf = nullptr;
    Mesh* m_mesh = nullptr;
    
    
    VolumeDataSource* m_vol_albedo = nullptr;
};


NORI_NAMESPACE_END
