/*
*/

#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Loader for Homogeneus
 */
class Homogeneous : public Medium {
public:
    Homogeneous(const PropertyList &propList) {
        // Here load other stuff:
        mu_a = propList.getFloat("mu_a", 0.1f); //("radiance", Color3f(1.f));
        mu_s = propList.getFloat("mu_s", 0.6f);
        mu_t = mu_s + mu_a;
        std::cout << " Mu_t : " << mu_t <<"\n";
    }

    const float Transmittance(Point3f x0, Point3f xz) const {
        float res = std::exp(-mu_t * (x0 - xz).norm());
        return res;
    }

    const float getScatteringCoeficient() const{
        return mu_s;
    }

    const bool sampleBetween(Point3f x, Point3f xz, float rnd, MediumIntersection& medIts) const {
        //Return if the medium is in between x and xz or not. If there's not, then there's no use in doing this.
        medIts.xz = xz;
        medIts.o = x;
        // This frame will be different for heterogeneus media or media where the phase function has orientation.
        medIts.shFrame = Frame(Vector3f(1, 0, 0));
        
        float t = log(rnd) / mu_t;
        medIts.pdf_xt = mu_t * exp(-mu_t * t);
        medIts.xt = x + t * (xz - x);
        return true;
    }


protected:
    float mu_a;
    float mu_s;
    float mu_t;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
