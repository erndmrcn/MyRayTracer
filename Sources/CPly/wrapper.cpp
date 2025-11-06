//
//  PLYReaderWrapper.cpp
//  C wrapper implementation for miniply PLYReader
//

#include "PLYReaderWrapper.h"
#include "miniply.h"

// Wrapper struct that holds the C++ object
struct PLYReaderWrapper {
    miniply::PLYReader reader;

    PLYReaderWrapper(const char* filename) : reader(filename) {}
};

// Lifecycle
PLYReaderRef ply_reader_create(const char* filename) {
    try {
        return new PLYReaderWrapper(filename);
    } catch (...) {
        return nullptr;
    }
}

void ply_reader_destroy(PLYReaderRef reader) {
    delete reader;
}

// Query methods
bool ply_reader_valid(PLYReaderRef reader) {
    return reader && reader->reader.valid();
}

bool ply_reader_has_element(PLYReaderRef reader) {
    return reader && reader->reader.has_element();
}

bool ply_reader_load_element(PLYReaderRef reader) {
    return reader && reader->reader.load_element();
}

void ply_reader_next_element(PLYReaderRef reader) {
    if (reader) {
        reader->reader.next_element();
    }
}

// Element info
bool ply_reader_element_is(PLYReaderRef reader, const char* name) {
    return reader && reader->reader.element_is(name);
}

uint32_t ply_reader_num_rows(PLYReaderRef reader) {
    return reader ? reader->reader.num_rows() : 0;
}

// Property finding
bool ply_reader_find_pos(PLYReaderRef reader, uint32_t* propIdxs) {
    return reader && reader->reader.find_pos(propIdxs);
}

bool ply_reader_find_texcoord(PLYReaderRef reader, uint32_t* propIdxs) {
    return reader && reader->reader.find_texcoord(propIdxs);
}

bool ply_reader_find_indices(PLYReaderRef reader, uint32_t* propIdxs) {
    return reader && reader->reader.find_indices(propIdxs);
}

bool ply_reader_find_normals(PLYReaderRef reader, uint32_t* propIdxs) {
    return reader && reader->reader.find_normal(propIdxs);
}

// Data extraction
bool ply_reader_extract_properties(PLYReaderRef reader,
                                   const uint32_t* propIdxs,
                                   uint32_t numProps,
                                   int destType,
                                   void* dest) {
    if (!reader) return false;
    return reader->reader.extract_properties(propIdxs, numProps,
                                            static_cast<miniply::PLYPropertyType>(destType),
                                            dest);
}

uint32_t ply_reader_sum_of_list_counts(PLYReaderRef reader, uint32_t propIdx) {
    return reader ? reader->reader.sum_of_list_counts(propIdx) : 0;
}

bool ply_reader_extract_list_property(PLYReaderRef reader,
                                      uint32_t propIdx,
                                      int destType,
                                      void* dest) {
    if (!reader) return false;
    return reader->reader.extract_list_property(propIdx,
                                               static_cast<miniply::PLYPropertyType>(destType),
                                               dest);
}

// Triangulation
bool ply_reader_requires_triangulation(PLYReaderRef reader, uint32_t propIdx) {
    return reader && reader->reader.requires_triangulation(propIdx);
}

uint32_t ply_reader_num_triangles(PLYReaderRef reader, uint32_t propIdx) {
    return reader ? reader->reader.num_triangles(propIdx) : 0;
}

bool ply_reader_extract_triangles(PLYReaderRef reader,
                                  uint32_t propIdx,
                                  const float* pos,
                                  uint32_t numVerts,
                                  int destType,
                                  void* dest) {
    if (!reader) return false;
    return reader->reader.extract_triangles(propIdx, pos, numVerts,
                                           static_cast<miniply::PLYPropertyType>(destType),
                                           dest);
}
