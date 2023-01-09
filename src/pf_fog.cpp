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
        //std::cout << " PF CREATEDDDD!!!!\n";
        //m_radiance = new ConstantSpectrumTexture(props.getColor("radiance", Color3f(1.f)));
    }

    /// Evaluate the BRDF model
    Color3f eval(const PFQueryRecord &bRec) const {
        return m_cte_albedo * INV_FOURPI;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const PFQueryRecord&bRec) const {
        // This will be INV_FOURPI
        return Warp::squareToUniformSpherePdf(bRec.wo);
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(PFQueryRecord&bRec, const Point2f &sample) const {
        bRec.wo = Warp::squareToUniformSphere(sample);
        float pdf_sample = pdf(bRec);
        // m_cte_albedo / INV_FOUR_PI
        return eval(bRec) / pdf_sample;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse with cte albedo\n");
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        /*switch (obj->getClassType()) {
        default:
            throw NoriException("Diffuse::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }*/
        throw NoriException("Diffuse::addChild(<%s>) is not supported!",
            classTypeName(obj->getClassType()));
    }


    EClassType getClassType() const { return EPhaseFunction; }
private:
    Color3f m_cte_albedo;
};

NORI_REGISTER_CLASS(PF_Fog, "pf_fog");
NORI_NAMESPACE_END
