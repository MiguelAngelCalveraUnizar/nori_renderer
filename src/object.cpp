/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

void NoriObject::addChild(NoriObject *, const std::string& name) {
    throw NoriException(
        "NoriObject::addChild() is not implemented for objects of type '%s'!",
        classTypeName(getClassType()));
}

void NoriObject::activate() { /* Do nothing */ }
void NoriObject::setParent(NoriObject *) { /* Do nothing */ }

std::map<std::string, NoriObjectFactory::Constructor> *NoriObjectFactory::m_constructors = nullptr;

void NoriObjectFactory::registerClass(const std::string &name, const Constructor &constr) {
    if (!m_constructors)
        m_constructors = new std::map<std::string, NoriObjectFactory::Constructor>();
    (*m_constructors)[name] = constr;
}

// Things from mitsuba:

void NoriObject::incRef() const {

    _InterlockedIncrement(&m_refCount);

}

void NoriObject::decRef(bool autoDeallocate) const {
    bool deallocate = false; //autoDeallocate
    int count = _InterlockedDecrement(&m_refCount);
    if(count >= 0) std::cout<<"Reference count is below zero!\n";
    if (count == 0 && deallocate) {
        std::cout << "Reference being deleted for some reason!\n";
        delete this;
    }
}



NORI_NAMESPACE_END
