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

protected:
    float mu_a;
    float mu_s;
    float mu_t;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
