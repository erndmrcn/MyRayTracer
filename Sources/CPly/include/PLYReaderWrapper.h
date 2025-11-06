//
//  PLYReaderWrapper.h
//  RayTracer
//
//  Created by Eren Demircan on 29.10.2025.
//

//
//  PLYReaderWrapper.h
//  C wrapper for miniply PLYReader
//
//  This provides a C API that Swift can safely call
//

#ifndef PLYReaderWrapper_h
#define PLYReaderWrapper_h

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer to hide C++ implementation
typedef struct PLYReaderWrapper* PLYReaderRef;

// Lifecycle
PLYReaderRef ply_reader_create(const char* filename);
void ply_reader_destroy(PLYReaderRef reader);

// Query methods
bool ply_reader_valid(PLYReaderRef reader);
bool ply_reader_has_element(PLYReaderRef reader);
bool ply_reader_load_element(PLYReaderRef reader);
void ply_reader_next_element(PLYReaderRef reader);

// Element info
bool ply_reader_element_is(PLYReaderRef reader, const char* name);
uint32_t ply_reader_num_rows(PLYReaderRef reader);

// Property finding
bool ply_reader_find_pos(PLYReaderRef reader, uint32_t* propIdxs);
bool ply_reader_find_texcoord(PLYReaderRef reader, uint32_t* propIdxs);
bool ply_reader_find_indices(PLYReaderRef reader, uint32_t* propIdxs);
bool ply_reader_find_normals(PLYReaderRef reader, uint32_t* propIdxs);

// Data extraction
bool ply_reader_extract_properties(PLYReaderRef reader,
                                   const uint32_t* propIdxs,
                                   uint32_t numProps,
                                   int destType,  // PLYPropertyType as int
                                   void* dest);

uint32_t ply_reader_sum_of_list_counts(PLYReaderRef reader, uint32_t propIdx);
bool ply_reader_extract_list_property(PLYReaderRef reader,
                                      uint32_t propIdx,
                                      int destType,
                                      void* dest);

// Triangulation
bool ply_reader_requires_triangulation(PLYReaderRef reader, uint32_t propIdx);
uint32_t ply_reader_num_triangles(PLYReaderRef reader, uint32_t propIdx);
bool ply_reader_extract_triangles(PLYReaderRef reader,
                                  uint32_t propIdx,
                                  const float* pos,
                                  uint32_t numVerts,
                                  int destType,
                                  void* dest);

#ifdef __cplusplus
}
#endif

#endif /* PLYReaderWrapper_h */
