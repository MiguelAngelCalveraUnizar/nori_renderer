/*
*/

#pragma once
#include <mitsuba/mmap.h>
#include <nori/object.h>
#include <nori/bbox.h>

#ifndef DOUBLE_PRECISION
#define DOUBLE_PRECISION
typedef double Float;
#endif

NORI_NAMESPACE_BEGIN

class VolumeDataSource : public NoriObject {

public:
    enum EVolumeType {
        EDensity = 0,
        EAlbedo = 1,
        EOrientation = 2,
        ENone = -1
    };

    // Is it a densities volume?
    bool isDensity() const {
        return m_type == EDensity;
    };

    /// Look up a floating point density value by position
    virtual Float lookupDensity(const Point3f &p) const = 0;
    
    // Is it a albedo volume?
    bool isAlbedo() const {
        return m_type == EAlbedo;
    };

    /// Look up a spectrum value by position
    virtual Color3f lookupAlbedo(const Point3f &p) const = 0;

    // Is it a orientations volume?
    bool isOrientation() const {
        return m_type == EOrientation;
    };

    /// Look up a vector orientation by position
    virtual Vector3f lookupOrientation(const Point3f &p) const = 0;

    ///**
    // * \brief Return the recommended step size for numerical
    // * integration or inifinity if this is not known/applicable
    // */
    virtual float getStepSize() const = 0;

    ///**
    // * \brief Return the maximum floating point value that
    // * could be returned by \ref lookupFloat.
    // *
    // * This is useful when implementing Woodcock-Tracking.
    // */
    virtual float getMaximumFloatValue() const = 0;

    EClassType getClassType() const { return EVolume; }
    //TODO CHANGE
protected:
    EVolumeType m_type = ENone;
    BoundingBox3f m_aabb;
};

NORI_NAMESPACE_END
