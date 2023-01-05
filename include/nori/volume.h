/*
    CHANGE THIS TOD
*/

//#pragma once
//#if !defined(__MITSUBA_RENDER_VOLUME_H_)
//#define __MITSUBA_RENDER_VOLUME_H_

//#include <mitsuba/core/cobject.h>
//#include <mitsuba/core/aabb.h>

#include <nori/object.h>


//MTS_NAMESPACE_BEGIN
NORI_NAMESPACE_BEGIN


/**
 * \brief Generalized source of volumetric information
 * \ingroup librender
 */
class VolumeDataSource : public NoriObject {
public:
    /// Serialize to a binary data stream.
    //virtual void serialize(Stream *stream, InstanceManager *manager) const;

    /// Return the bounding box
    //inline const AABB &getAABB() const {
    //    return m_aabb;
    //}

    /// Are float-valued lookups permitted?
    virtual bool supportsFloatLookups() const;

    /// Look up a floating point value by position
    virtual float lookupFloat(const Point3f &p) const;

    /// Are spectrum-valued lookups permitted?
    virtual bool supportsSpectrumLookups() const;

    /// Look up a spectrum value by position
    //virtual Spectrum lookupSpectrum(const Point &p) const;

    /// Are vector-valued lookups permitted?
    virtual bool supportsVectorLookups() const;

    /// Look up a vector value by position
    virtual Vector3f lookupVector(const Point3f &p) const;

    /**
     * \brief Return the recommended step size for numerical
     * integration or inifinity if this is not known/applicable
     */
    virtual float getStepSize() const = 0;

    /**
     * \brief Return the maximum floating point value that
     * could be returned by \ref lookupFloat.
     *
     * This is useful when implementing Woodcock-Tracking.
     */
    virtual float getMaximumFloatValue() const = 0;
    //MTS_DECLARE_CLASS()
protected:
    /// Virtual destructor
    virtual ~VolumeDataSource();

    /// Protected constructor
    VolumeDataSource(const PropertyList &props);

    /// Unserialize from a binary data stream
    //VolumeDataSource(Stream *stream, InstanceManager *manager);
//protected:
    //AABB m_aabb;
};

//MTS_NAMESPACE_END
NORI_NAMESPACE_END
//#endif /* __MITSUBA_RENDER_VOLUME_H_ */
