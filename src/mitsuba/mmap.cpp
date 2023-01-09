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

#include <mitsuba/mmap.h>
#if defined(__LINUX__) || defined(__OSX__)
# include <sys/mman.h>
# include <fcntl.h>
#elif defined(__WINDOWS__)
# include <windows.h>
#endif

namespace fs = filesystem;
#define __WINDOWS__
NORI_NAMESPACE_BEGIN

struct MemoryMappedFile::MemoryMappedFilePrivate {
    fs::path filename;

#if defined(__WINDOWS__)
    HANDLE file;
    HANDLE fileMapping;
#endif
    size_t size;
    void *data;
    bool readOnly;
    bool temp;

    MemoryMappedFilePrivate(const fs::path &f = "", size_t s = 0)
        : filename(f), size(s), data(NULL), readOnly(false), temp(false) {}

    void create() {
        #if defined(__LINUX__) || defined(__OSX__)
            int fd = open(filename.string().c_str(), O_RDWR | O_CREAT | O_TRUNC, 0664);
            if (fd == -1)
                throw NoriException("Could not open \"%s\"!", filename.string().c_str());
            int result = lseek(fd, size-1, SEEK_SET);
            if (result == -1)
                throw NoriException("Could not set file size of \"%s\"!", filename.string().c_str());
            result = write(fd, "", 1);
            if (result != 1)
                throw NoriException("Could not write to \"%s\"!", filename.string().c_str());
            data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (data == NULL)
                throw NoriException("Could not map \"%s\" to memory!", filename.string().c_str());
            if (close(fd) != 0)
                throw NoriException("close(): unable to close file!");
        #elif defined(__WINDOWS__)
            file = CreateFile(filename.str().c_str(), GENERIC_WRITE | GENERIC_READ,
                FILE_SHARE_READ, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
            if (file == INVALID_HANDLE_VALUE)
                throw NoriException("Could not open \"%s\": %s", filename.str().c_str());
            fileMapping = CreateFileMapping(file, NULL, PAGE_READWRITE, 0,
                static_cast<DWORD>(size), NULL);
            if (fileMapping == NULL)
                throw NoriException("CreateFileMapping: Could not map \"%s\" to memory: %s",
                    filename.str().c_str());
            data = (void *) MapViewOfFile(fileMapping, FILE_MAP_WRITE, 0, 0, 0);
            if (data == NULL)
                throw NoriException("MapViewOfFile: Could not map \"%s\" to memory: %s",
                    filename.str().c_str());
        #endif
        readOnly = false;
    }

    void createTemp() {
        readOnly = false;
        temp = true;

        #if defined(__LINUX__) || defined(__OSX__)
            char *path = strdup("/tmp/mitsuba_XXXXXX");
            int fd = mkstemp(path);
            if (fd == -1)
                throw NoriException("Unable to create temporary file (1): %s", strerror(errno));
            filename = path;
            free(path);

            int result = lseek(fd, size-1, SEEK_SET);
            if (result == -1)
                throw NoriException("Could not set file size of \"%s\"!", filename.string().c_str());
            result = write(fd, "", 1);
            if (result != 1)
                throw NoriException("Could not write to \"%s\"!", filename.string().c_str());

            data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (data == NULL)
                throw NoriException("Could not map \"%s\" to memory!", filename.string().c_str());

            if (close(fd) != 0)
                throw NoriException("close(): unable to close file!");
        #elif defined(__WINDOWS__)
            WCHAR tempPath[MAX_PATH];
            WCHAR tempFilename[MAX_PATH];

            unsigned int ret = GetTempPathW(MAX_PATH, tempPath);
            if (ret == 0 || ret > MAX_PATH)
                throw NoriException("GetTempPath failed(): %s");

            ret = GetTempFileNameW(tempPath, L"mitsuba", 0, tempFilename);
            if (ret == 0)
                throw NoriException("GetTempFileName failed(): %s");

            file = CreateFileW(tempFilename, GENERIC_READ | GENERIC_WRITE,
                0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

            if (file == INVALID_HANDLE_VALUE)
                throw NoriException("Error while trying to create temporary file: %s");

            filename = fs::path(tempFilename);

            fileMapping = CreateFileMapping(file, NULL, PAGE_READWRITE, 0,
                static_cast<DWORD>(size), NULL);
            if (fileMapping == NULL)
                throw NoriException("CreateFileMapping: Could not map \"%s\" to memory:",
                    filename.str().c_str());
            data = (void *) MapViewOfFile(fileMapping, FILE_MAP_WRITE, 0, 0, 0);
            if (data == NULL)
                throw NoriException("MapViewOfFile: Could not map \"%s\" to memory:",
                    filename.str().c_str());
        #endif
    }

    void map() {
        if (!filename.exists())
            throw NoriException("The file \"%s\" does not exist!", filename.str().c_str());
        size = (size_t) filename.file_size();

        #if defined(__LINUX__) || defined(__OSX__)
            int fd = open(filename.string().c_str(), readOnly ? O_RDONLY : O_RDWR);
            if (fd == -1)
                throw NoriException("Could not open \"%s\"!", filename.str().c_str());
            data = mmap(NULL, size, PROT_READ | (readOnly ? 0 : PROT_WRITE), MAP_SHARED, fd, 0);
            if (data == NULL)
                throw NoriException("Could not map \"%s\" to memory!", filename.str().c_str());
            if (close(fd) != 0)
                throw NoriException("close(): unable to close file!");
        #elif defined(__WINDOWS__)
            file = CreateFile(filename.str().c_str(), GENERIC_READ | (readOnly ? 0 : GENERIC_WRITE),
                FILE_SHARE_WRITE|FILE_SHARE_READ, NULL, OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL, NULL);
            if (file == INVALID_HANDLE_VALUE)
                throw NoriException("Could not open \"%s\":", filename.str().c_str());
            fileMapping = CreateFileMapping(file, NULL, readOnly ? PAGE_READONLY : PAGE_READWRITE, 0, 0, NULL);
            if (fileMapping == NULL)
                throw NoriException("CreateFileMapping: Could not map \"%s\" to memory:",
                    filename.str().c_str());
            data = (void *) MapViewOfFile(fileMapping, readOnly ? FILE_MAP_READ : FILE_MAP_WRITE, 0, 0, 0);
            if (data == NULL)
                throw NoriException("MapViewOfFile: Could not map \"%s\" to memory:",
                    filename.str().c_str());
        #endif
    }

    void unmap() {
        std::cout<<"Unmapping from memory: "<<filename.str().c_str();

        #if defined(__LINUX__) || defined(__OSX__)
            if (temp) {
                /* Temporary file that will be deleted in any case:
                   invalidate dirty pages to avoid a costly flush to disk */
                int retval = msync(data, size, MS_INVALIDATE);
                if (retval != 0)
                    throw NoriException("munmap(): unable to unmap memory: %s", strerror(errno));
            }

            int retval = munmap(data, size);
            if (retval != 0)
                throw NoriException("munmap(): unable to unmap memory: %s", strerror(errno));
        #elif defined(__WINDOWS__)
            if (!UnmapViewOfFile(data))
                throw NoriException("UnmapViewOfFile(): unable to unmap memory:");
            if (!CloseHandle(fileMapping))
                throw NoriException("CloseHandle(): unable to close file mapping:");
            if (!CloseHandle(file))
                throw NoriException("CloseHandle(): unable to close file: ");
        #endif

        if (temp) {
            try {
                filename.remove_file();
                //fs::remove(filename);
            } catch (...) {
                throw NoriException("unmap(): Unable to delete file \"%s\"", filename.str().c_str());
            }
        }

        data = NULL;
        size = 0;
    }
};

MemoryMappedFile::MemoryMappedFile()
    : d(new MemoryMappedFilePrivate()) { }

MemoryMappedFile::MemoryMappedFile(const fs::path &filename, size_t size)
    : d(new MemoryMappedFilePrivate(filename, size)) {
    std::cout<< "Creating memory-mapped file \"%s\" (%s).."<<
        filename.filename().c_str() <<" ;"<< memString(d->size).c_str();
    d->create();
}


MemoryMappedFile::MemoryMappedFile(const fs::path &filename, bool readOnly)
    : d(new MemoryMappedFilePrivate(filename)) {
    d->readOnly = readOnly;
    d->map();
    std::cout << "Creating memory-mapped file \"%s\" (%s).." <<
        filename.filename().c_str() << " ;" << memString(d->size).c_str()<<"\n";
}

MemoryMappedFile::~MemoryMappedFile() {
    if (d->data) {
        try {
            d->unmap();
        } catch (std::exception &e) {
            /* Don't throw exceptions from a constructor */
            std::cout<< e.what();
        }
    }
}

void MemoryMappedFile::resize(size_t size) {
    if (!d->data)
        throw NoriException("Internal error in MemoryMappedFile::resize()!");
    bool temp = d->temp;
    d->temp = false;
    d->unmap();
    d->filename.resize_file(size);
    //fs::resize_file(d->filename, size);
    d->size = size;
    d->map();
    d->temp = temp;
}

void *MemoryMappedFile::getData() {
    return d->data;
}

/// Return a pointer to the file contents in memory (const version)
const void *MemoryMappedFile::getData() const {
    return d->data;
}

size_t MemoryMappedFile::getSize() const {
    return d->size;
}

bool MemoryMappedFile::isReadOnly() const {
    return d->readOnly;
}

const fs::path &MemoryMappedFile::getFilename() const {
    return d->filename;
}

ref<MemoryMappedFile> MemoryMappedFile::createTemporary(size_t size) {
    ref<MemoryMappedFile> result = new MemoryMappedFile();
    result->d->size = size;
    result->d->createTemp();
    return result;
}

std::string MemoryMappedFile::toString() const {
    std::ostringstream oss;
    oss << "MemoryMappedFile[filename=\""
        << d->filename.str() << "\", size="
        << memString(d->size) << "]";
    return oss.str();
}

//NORI_IMPLEMENT_CLASS(MemoryMappedFile, false, Object)
NORI_NAMESPACE_END
