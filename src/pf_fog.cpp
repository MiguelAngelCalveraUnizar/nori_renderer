/*
*/

#include <nori/pf.h>
#include <nori/frame.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class PF_Fog : public PhaseFunction {
public:
    PF_Fog(const PropertyList &propList) {
        m_cte_albedo = Color3f(1,1,1); //White fog
        m_g = propList.getFloat("g", 0.0f);
    }

    /// Evaluate the BRDF model
    Color3f eval(const PFQueryRecord &bRec) const {
        return m_cte_albedo * pdf(bRec);
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const PFQueryRecord&bRec) const {
        float g = m_g;
        float denom = 1 + g * g + 2 * g * Frame::cosTheta(bRec.wo);
        return INV_FOURPI * (1 - g * g) / (denom * std::sqrt(denom));
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(PFQueryRecord&bRec, const Point2f &sample) const {
        float cosTheta;
        float g = m_g;
        if (std::abs(g) < 1e-3)
            cosTheta = 1 - 2 * sample[0];
        else {
            float sqrTerm = (1 - g * g) /
                (1 - g + 2 * g * sample[0]);
            cosTheta = (1 + g * g - sqrTerm * sqrTerm) / (2 * g);
        }   

        float sinTheta = std::sqrt(std::max((float)0,
            1 - cosTheta * cosTheta));
        float phi = 2 * M_PI * sample[1];
        
        bRec.wo = Vector3f(sinTheta * std::cos(phi),
            sinTheta * std::sin(phi),
            cosTheta);

        //float pdf_sample = pdf(bRec);
        float pdf_sample = 1; //We assume we sample perfectly the values -> No need to weight the sample

        return eval(bRec) / pdf_sample;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse with cte albedo\n");
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        throw NoriException("Diffuse::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
    }


    EClassType getClassType() const { return EPhaseFunction; }
private:
    Color3f m_cte_albedo;
    float m_g;
};

NORI_REGISTER_CLASS(PF_Fog, "pf_fog");
NORI_NAMESPACE_END
