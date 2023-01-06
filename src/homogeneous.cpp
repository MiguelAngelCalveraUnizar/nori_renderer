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


    /*
    * This function gives the trasnmittance between 2 points
    * This assumes the medium is actually between the 2 points you selected, if you pick a new 
    * random point without checking if it's on the bounding box of the medium or not, it will not work
    */
    virtual const float Transmittance(Point3f x, Point3f xz) const {
        float res = std::exp(-mu_t * (x - xz).norm());
        return res;
    }

    virtual const float getScatteringCoeficient() const{
        return mu_s;
    }

    /* 
    * Void function 
    * Adds to medIts a .xt and the corresponding pdf_xt as well as the .shFrame of the particle (always the same as we have isotropic homogeneus media) 
    */
    virtual void sampleBetween(float rnd, MediumIntersection& medIts) const {
        Point3f xz = medIts.xz;
        Point3f x = medIts.x;
        // This frame will be different for heterogeneus media or media where the phase function has orientation.
        medIts.shFrame = Frame(Vector3f(1, 0, 0));
        float t = log(rnd) / mu_t;
        medIts.pdf_xt = mu_t * exp(-mu_t * t);
        medIts.xt = x + t * (xz - x);
    }


protected:
    float mu_a;
    float mu_s;
    float mu_t;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
