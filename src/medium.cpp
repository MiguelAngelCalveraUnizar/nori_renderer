/*
*/

#include <nori/medium.h>
#include <nori/bbox.h>
#include <nori/pf.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

// I have doubts of:
#include <nori/mesh.h>


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

void Medium::sampleBetween(float rnd, MediumIntersection& medIts) const {
    std::cout << "Medium: sampleBetween not defined for parent class!";
    throw NoriException(
        "Medium: sampleBetween not defined for parent class!");
}


void Medium::addChild(NoriObject* obj, const std::string& name) {
    switch (obj->getClassType()) {
    case EPhaseFunction: {
        if (m_pf)
            throw NoriException(
                "Medium: tried to register multiple PF instances!");
        m_pf = static_cast<PhaseFunction*>(obj);
        std::cout << " PF Aded as child to the Medium\n";
        break;
    }
    case EMesh: {
        if (m_mesh)
            throw NoriException(
                "Medium: tried to register multiple Mesh instances!");
        m_mesh = static_cast<Mesh*>(obj);
        std::cout << " Mesh Aded as child to the Medium \n";
        break;
    }
    case EVolume: {
        std::cout << " Volume Aded as child to the Medium \n";
        if  (!m_vol_density) {
            m_vol_density = static_cast<VolumeDataSource*>(obj);
            std::cout << "1 Albedo Aded as child to the Medium \n";
        }
        else if (!m_vol_albedo) {
            m_vol_albedo = static_cast<VolumeDataSource*>(obj);
            std::cout << "2 Density Aded as child to the Medium \n";
        }
        else if (!m_vol_orientation) {
            m_vol_orientation = static_cast<VolumeDataSource*>(obj);
            std::cout << "3 Orientation Aded as child to the Medium \n";
        }
        else {
            std::cout << " Added volume as a child to medium WITHOUT NAME \n";
        }
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
