/*
*/

#include <nori/medium.h>
#include <nori/bbox.h>
#include <nori/pf.h>
#include <nori/warp.h>
#include <Eigen/Geometry>



NORI_NAMESPACE_BEGIN

Medium::Medium() { }

Medium::~Medium() {
    delete m_pf;
}

void Medium::activate() {
    if (!m_pf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_pf = static_cast<PhaseFunction*>(
            NoriObjectFactory::createInstance("pf_fog", PropertyList()));
    }
}


void Medium::addChild(NoriObject *obj, const std::string& name) {
    switch (obj->getClassType()) {
        case EPhaseFunction: {
            if (m_pf)
                throw NoriException(
                    "Medium: tried to register multiple PF instances!");
            m_pf = static_cast<PhaseFunction*>(obj);
            std::cout << " PF Aded as child \n";
            break;
        }
        default:
            throw NoriException("Medium::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Medium::toString() const {
    return tfm::format(
        "Medium[\n"
        "  name = \"%s\",\n"
        "]",
        m_name
    );
}

std::string MediumIntersection::toString() const {
    if (!medium)
        return "MediumIntersection[invalid]";

    return tfm::format(
        "MediumIntersection[\n"
        "  o = %s,\n"
        "  xt = %s,\n"
        "  xz = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        o.toString(),
        xt.toString(),
        xz.toString(),
        indent(shFrame.toString()),
        medium ? medium->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
