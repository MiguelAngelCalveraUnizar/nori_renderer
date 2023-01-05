/*

*/


#pragma once
#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Convenience data structure used to pass multiple
 * parameters to the evaluation and sampling routines in \ref BSDF
 */
struct PFQueryRecord {
    /// Incident direction (in the local frame)
    Vector3f wi;

    /// Outgoing direction (in the local frame)
    Vector3f wo;

    /// Create a new record for sampling the BSDF
    PFQueryRecord(const Vector3f &wi)
        : wi(wi){ }

    /// Create a new record for querying the BSDF
    PFQueryRecord(const Vector3f &wi, const Vector3f &wo)
        : wi(wi), wo(wo){ }
};

/**
 * \brief Superclass of all PhaseFunction
 */
class PhaseFunction : public NoriObject {
public:
    /**
     * \brief Sample the PhaseFunction and return the importance weight (i.e. the
     * value of the PhaseFunction divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param bRec    A PhaseFunction query record
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return The PhaseFunction value divided by the probability density of the sample
     *         sample. A zero value means that sampling
     *         failed.
     */
    virtual Color3f sample(PFQueryRecord&bRec, const Point2f &sample) const = 0;

    /**
     * \brief Evaluate the PhaseFunction for a pair of directions and measure
     * specified in \code bRec
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     * \return
     *     The PhaseFunction value, evaluated for each color channel
     */
    virtual Color3f eval(const PFQueryRecord&bRec) const = 0;

    /**
     * \brief Compute the probability of sampling \c bRec.wo
     * (conditioned on \c bRec.wi).
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param bRec
     *     A record with detailed information on the PhaseFunction query
     *
     * \return
     *     A probability/density value expressed with respect
     *     to the specified measure
     */

    virtual float pdf(const PFQueryRecord &bRec) const = 0;

    /**
     * \brief Return the type of object (i.e. Medium/PhaseFunction/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EPhaseFunction; }

    ///**
    // * \brief Return whether or not this PhaseFunction is diffuse. This
    // * is primarily used by photon mapping to decide whether
    // * or not to store photons on a surface
    // */
    //virtual bool isDiffuse() const { return false; }
};

NORI_NAMESPACE_END
