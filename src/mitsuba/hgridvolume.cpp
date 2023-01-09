/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
    
    This file was edited to fit inside Nori, please go to the original
    for a good way of doing things, don't take this one as the example
*/

#include <nori/volume.h>
#include <mitsuba/mstream.h>
#include <filesystem/resolver.h>
#include <nori/proplist.h>
#include <string>

NORI_NAMESPACE_BEGIN

/**
 * This class implements a two-layer hierarchical grid
 * using 'gridvolume'-based files. It loads a dictionary
 * and then proceeds to map volume data into memory
 */
class HierarchicalGridDataSource : public VolumeDataSource {
public:
    HierarchicalGridDataSource(const PropertyList &props){
        int type_read = props.getInteger("voltype", -1);

        if (type_read == (int)EDensity) {
            m_type = EDensity;
        }
        else if (type_read == (int)EAlbedo) {
            m_type = EAlbedo;
        }
        else if (type_read == (int)EOrientation) {
            m_type = EOrientation;
        }
        else {
            throw NoriException("Not a valid voltype set in XML!");
        }


        //m_volumeToWorld = props.getTransform("toWorld", Transform());
        m_prefix = props.getString("prefix");
        m_postfix = props.getString("postfix");
        std::string filename = props.getString("filename");
        
        
        
        loadDictionary(filename);
    }

    /*HierarchicalGridDataSource(Stream *stream, InstanceManager *manager)
    : VolumeDataSource(stream, manager) {
        m_volumeToWorld = Transform(stream);
        std::string filename = stream->readString();
        m_prefix = stream->readString();
        m_postfix = stream->readString();
        loadDictionary(filename);
    }*/

    ~HierarchicalGridDataSource() {
        size_t nCells = m_res[0] * m_res[1] * m_res[2];
        for (size_t i=0; i<nCells; ++i) {
            if (m_blocks[i] != NULL)
                m_blocks[i]->decRef();
        }
        delete[] m_blocks;
    }

    /*void serialize(Stream *stream, InstanceManager *manager) const {
        VolumeDataSource::serialize(stream, manager);

        m_volumeToWorld.serialize(stream);
        stream->writeString(m_filename);
        stream->writeString(m_prefix);
        stream->writeString(m_postfix);
    }*/

    void loadDictionary(const std::string &filename) {
        //fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
        fs::path resolved =
            getFileResolver()->resolve(filename);
        std::cout<<"Loading hierarchical grid dictionary "<< filename.c_str();

        //ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
        MemoryMappedFile * m_mmap = new MemoryMappedFile(resolved);
        ref<MemoryStream> stream = new MemoryStream(m_mmap->getData(), m_mmap->getSize());
        stream->setByteOrder(Stream::ELittleEndian);
        Float xmin = stream->readSingle(), ymin = stream->readSingle(), zmin = stream->readSingle();
        Float xmax = stream->readSingle(), ymax = stream->readSingle(), zmax = stream->readSingle();
        BoundingBox3f aabb = BoundingBox3f(Point3f(xmin, ymin, zmin), Point3f(xmax, ymax, zmax));
        //m_res = Vector3i(stream);
        int temp_x = stream->readInt();
        int temp_y = stream->readInt();
        int temp_z = stream->readInt();
        m_res = Vector3i(temp_x, temp_y, temp_z);
        m_filename = filename;
        size_t nCells = m_res[0] * m_res[1] * m_res[2];
        m_blocks = new VolumeDataSource*[nCells];
        memset(m_blocks, 0, nCells*sizeof(VolumeDataSource *));
        Vector3f extents = aabb.getExtents();
        //m_worldToVolume = m_volumeToWorld.inverse();
        m_worldToGrid = Transform::scale(Vector3f(
                (m_res[0]) / extents[0],
                (m_res[1]) / extents[1],
                (m_res[2]) / extents[2])
            ) * Transform::translate(-Vector3f(aabb.min)) ; //* m_worldToVolume
        
        std::cout << "m_worldToGrid: \n" << m_worldToGrid.toString();

        m_supportsFloatLookups = true;
        m_supportsVectorLookups = true;
        m_supportsSpectrumLookups = true;
        m_stepSize = std::numeric_limits<Float>::infinity();

        int numBlocks = 0;
        while (!stream->isEOF()) {
            //Vector3i block = Vector3i(stream);
            temp_x = stream->readInt();
            temp_y = stream->readInt();
            temp_z = stream->readInt();
            Vector3i block = Vector3i(temp_x, temp_y, temp_z);
            /* Assert(block[0] >= 0 && block[1] >= 0 && block[2] >= 0
                    && block[0] < m_res[0] && block[1] < m_res[1] && block[2] < m_res[2]);*/
            /*if (block[0] >= 0 && block[1] >= 0 && block[2] >= 0
                && block[0] < m_res[0] && block[1] < m_res[1] && block[2] < m_res[2]) {
                std::cout << "[ASSERTION] Vol description file has a block that doesn't correspond with anything \n";
                continue;
            }*/
            PropertyList props;
            /*result = NoriObjectFactory::createInstance(
                node.attribute("type").value(),
                propList*/
            std::string full_filename = formatString("%s%03i_%03i_%03i%s",
                m_prefix.c_str(), block[0], block[1], block[2], m_postfix.c_str());
            props.setString("filename", full_filename);
            std::cout << "Adding block: " << full_filename<<"\n";


            //std::cout << "Postfix: " << m_postfix.c_str() << "\n";
            //PropertyList props("gridVolume");
            /*props.setString("filename", formatString("%s%03i_%03i_%03i%s",
                        m_prefix.c_str(), block.x, block.y, block.z, m_postfix.c_str()));*/
            //props.setTransform("toWorld", m_volumeToWorld);
            //props.setBoolean("sendData", false);

            
            props.setInteger("voltype", m_type);

            NoriObject *obj = NoriObjectFactory::createInstance("gridVolume", props);
            
            VolumeDataSource *content = static_cast<VolumeDataSource*> (obj);

            /*VolumeDataSource *content = static_cast<VolumeDataSource *> (PluginManager::getInstance()->
                    createObject(MTS_CLASS(VolumeDataSource), props));*/
            content->activate();

            m_maxFloatValue = content->getMaximumFloatValue();
            m_blocks[(m_res[1] * block[2] + block[1]) * m_res[0] + block[0]] = content;
            m_stepSize = std::min(m_stepSize, (double)content->getStepSize());
            m_supportsVectorLookups = m_supportsVectorLookups && content->isOrientation();
            m_supportsFloatLookups = m_supportsFloatLookups && content->isDensity();
            m_supportsSpectrumLookups = m_supportsSpectrumLookups && content->isAlbedo();
            content->incRef();
            ++numBlocks;
        }
        std::cout << numBlocks << " blocks total," << aabb.toString().c_str() << ", stepSize=" << m_stepSize << ", resolution=" << m_res.toString().c_str()<<"\n";
        if (numBlocks == 701) {
            std:cout << "[CONGRATS] Everything is okay \n\n";
        }
        m_aabb.reset();
        for (int i = 0; i < 8; ++i) {
            //m_aabb.expandBy(m_volumeToWorld(aabb.getCorner(i)));
            m_aabb.expandBy(aabb.getCorner(i));
        }
            
    }

    bool supportsFloatLookups() const {
        return m_supportsFloatLookups;
    }

    bool supportsSpectrumLookups() const {
        return m_supportsSpectrumLookups;
    }

    bool supportsVectorLookups() const {
        return m_supportsVectorLookups;
    }

    float getStepSize() const {
        return (float)m_stepSize;
    }

    Float lookupDensity(const Point3f &_p) const {
        Point3f p = m_worldToGrid.transformAffine(Point3f(_p));
        const int x = (int)std::floor(p[0]),
              y = (int)std::floor(p[1]),
              z = (int)std::floor(p[2]);
        if (x < 0 || x >= m_res[0] ||
            y < 0 || y >= m_res[1] ||
            z < 0 || z >= m_res[2]) {
            //std::cout << "Ow....! Bad x,y,z";
            //std::cout << "-p affine xyz->"<<x<<"," << y << "," << z;
            //std::cout << "_pxyz->" << _p[0] << "," << _p[1] << "," << _p[2] << "\n";
            /*std::cout << "Outside the values allowed \n";*/
            //std::cout << "limits:" << m_res.toString();//<< "values: (" << x << "," << y << "," << z << ")";
            return 0.0f;
        }

        VolumeDataSource *block = m_blocks[((z * m_res[1]) + y) * m_res[0] + x];
        if (block == NULL) {
            //std::cout << "Ow....! doesn't exist";
            return 0.0f;
        }
        else {
            return block->lookupDensity(_p);
        }
    }

    Color3f lookupAlbedo(const Point3f& p) const {
        throw NoriException("There's no support for variable albedo!");
        return Color3f(0);
    };

    /*Spectrum lookupSpectrum(const Point3f &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
        const int x = math::floorToInt(p.x),
              y = math::floorToInt(p.y),
              z = math::floorToInt(p.z);
        if (x < 0 || x >= m_res.x ||
            y < 0 || y >= m_res.y ||
            z < 0 || z >= m_res.z)
            return Spectrum(0.0f);

        VolumeDataSource *block = m_blocks[((z * m_res.y) + y) * m_res.x + x];
        if (block == NULL)
            return Spectrum(0.0f);
        else
            return block->lookupSpectrum(_p);
    }*/

    Vector3f lookupOrientation(const Point3f &_p) const {
        const Point3f p = m_worldToGrid.transformAffine(_p);
        //const Point3f p = _p;
        const int x = (int)std::floor(p[0]),
            y = (int)std::floor(p[1]),
            z = (int)std::floor(p[2]);
        if (x < 0 || x >= m_res[0] ||
            y < 0 || y >= m_res[1] ||
            z < 0 || z >= m_res[2])
            return Vector3f(0.0f);

        VolumeDataSource* block = m_blocks[((z * m_res[1]) + y) * m_res[0] + x];
        if (block == NULL)
            return Vector3f(0);
        else
            return block->lookupOrientation(p);
    }

    float getMaximumFloatValue() const {
        return (float)m_maxFloatValue;
    }

    std::string toString() const {
        return "HgridVolume";
    }


protected:

    std::string formatString(const char* fmt, ...) {
        char tmp[512];
        va_list iterator;

        va_start(iterator, fmt);
        size_t size = _vscprintf(fmt, iterator) + 1;

        if (size >= sizeof(tmp)) {
            char* dest = new char[size];
            vsnprintf_s(dest, size, size - 1, fmt, iterator);
            va_end(iterator);
            std::string result(dest);
            delete[] dest;
            return result;
        }

        vsnprintf_s(tmp, size, size - 1, fmt, iterator);
        va_end(iterator);

        return std::string(tmp);
    }

    std::string m_filename, m_prefix, m_postfix;
    //Transform m_volumeToWorld;
    //Transform m_worldToVolume;
    Transform m_worldToGrid;
    VolumeDataSource **m_blocks;
    Vector3i m_res;
    size_t m_count;
    bool m_supportsFloatLookups;
    bool m_supportsSpectrumLookups;
    bool m_supportsVectorLookups;
    Float m_stepSize, m_maxFloatValue;
};

NORI_REGISTER_CLASS(HierarchicalGridDataSource, "HgridVolume");
NORI_NAMESPACE_END

