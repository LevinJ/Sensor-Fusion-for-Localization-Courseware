// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: scan_context/scan_contexts.proto

#ifndef PROTOBUF_scan_5fcontext_2fscan_5fcontexts_2eproto__INCLUDED
#define PROTOBUF_scan_5fcontext_2fscan_5fcontexts_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace scan_context_io {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
void protobuf_AssignDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
void protobuf_ShutdownFile_scan_5fcontext_2fscan_5fcontexts_2eproto();

class ScanContext;
class ScanContexts;

// ===================================================================

class ScanContext : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.ScanContext) */ {
 public:
  ScanContext();
  virtual ~ScanContext();

  ScanContext(const ScanContext& from);

  inline ScanContext& operator=(const ScanContext& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ScanContext& default_instance();

  void Swap(ScanContext* other);

  // implements Message ----------------------------------------------

  inline ScanContext* New() const { return New(NULL); }

  ScanContext* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ScanContext& from);
  void MergeFrom(const ScanContext& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ScanContext* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float data = 1;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 1;
  float data(int index) const;
  void set_data(int index, float value);
  void add_data(float value);
  const ::google::protobuf::RepeatedField< float >&
      data() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_data();

  // @@protoc_insertion_point(class_scope:scan_context_io.ScanContext)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > data_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fscan_5fcontexts_2eproto();

  void InitAsDefaultInstance();
  static ScanContext* default_instance_;
};
// -------------------------------------------------------------------

class ScanContexts : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.ScanContexts) */ {
 public:
  ScanContexts();
  virtual ~ScanContexts();

  ScanContexts(const ScanContexts& from);

  inline ScanContexts& operator=(const ScanContexts& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ScanContexts& default_instance();

  void Swap(ScanContexts* other);

  // implements Message ----------------------------------------------

  inline ScanContexts* New() const { return New(NULL); }

  ScanContexts* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ScanContexts& from);
  void MergeFrom(const ScanContexts& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ScanContexts* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 num_rings = 1;
  bool has_num_rings() const;
  void clear_num_rings();
  static const int kNumRingsFieldNumber = 1;
  ::google::protobuf::int32 num_rings() const;
  void set_num_rings(::google::protobuf::int32 value);

  // required int32 num_sectors = 2;
  bool has_num_sectors() const;
  void clear_num_sectors();
  static const int kNumSectorsFieldNumber = 2;
  ::google::protobuf::int32 num_sectors() const;
  void set_num_sectors(::google::protobuf::int32 value);

  // repeated .scan_context_io.ScanContext data = 3;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 3;
  const ::scan_context_io::ScanContext& data(int index) const;
  ::scan_context_io::ScanContext* mutable_data(int index);
  ::scan_context_io::ScanContext* add_data();
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::ScanContext >*
      mutable_data();
  const ::google::protobuf::RepeatedPtrField< ::scan_context_io::ScanContext >&
      data() const;

  // @@protoc_insertion_point(class_scope:scan_context_io.ScanContexts)
 private:
  inline void set_has_num_rings();
  inline void clear_has_num_rings();
  inline void set_has_num_sectors();
  inline void clear_has_num_sectors();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 num_rings_;
  ::google::protobuf::int32 num_sectors_;
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::ScanContext > data_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fscan_5fcontexts_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fscan_5fcontexts_2eproto();

  void InitAsDefaultInstance();
  static ScanContexts* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// ScanContext

// repeated float data = 1;
inline int ScanContext::data_size() const {
  return data_.size();
}
inline void ScanContext::clear_data() {
  data_.Clear();
}
inline float ScanContext::data(int index) const {
  // @@protoc_insertion_point(field_get:scan_context_io.ScanContext.data)
  return data_.Get(index);
}
inline void ScanContext::set_data(int index, float value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:scan_context_io.ScanContext.data)
}
inline void ScanContext::add_data(float value) {
  data_.Add(value);
  // @@protoc_insertion_point(field_add:scan_context_io.ScanContext.data)
}
inline const ::google::protobuf::RepeatedField< float >&
ScanContext::data() const {
  // @@protoc_insertion_point(field_list:scan_context_io.ScanContext.data)
  return data_;
}
inline ::google::protobuf::RepeatedField< float >*
ScanContext::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:scan_context_io.ScanContext.data)
  return &data_;
}

// -------------------------------------------------------------------

// ScanContexts

// required int32 num_rings = 1;
inline bool ScanContexts::has_num_rings() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ScanContexts::set_has_num_rings() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ScanContexts::clear_has_num_rings() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ScanContexts::clear_num_rings() {
  num_rings_ = 0;
  clear_has_num_rings();
}
inline ::google::protobuf::int32 ScanContexts::num_rings() const {
  // @@protoc_insertion_point(field_get:scan_context_io.ScanContexts.num_rings)
  return num_rings_;
}
inline void ScanContexts::set_num_rings(::google::protobuf::int32 value) {
  set_has_num_rings();
  num_rings_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.ScanContexts.num_rings)
}

// required int32 num_sectors = 2;
inline bool ScanContexts::has_num_sectors() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ScanContexts::set_has_num_sectors() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ScanContexts::clear_has_num_sectors() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ScanContexts::clear_num_sectors() {
  num_sectors_ = 0;
  clear_has_num_sectors();
}
inline ::google::protobuf::int32 ScanContexts::num_sectors() const {
  // @@protoc_insertion_point(field_get:scan_context_io.ScanContexts.num_sectors)
  return num_sectors_;
}
inline void ScanContexts::set_num_sectors(::google::protobuf::int32 value) {
  set_has_num_sectors();
  num_sectors_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.ScanContexts.num_sectors)
}

// repeated .scan_context_io.ScanContext data = 3;
inline int ScanContexts::data_size() const {
  return data_.size();
}
inline void ScanContexts::clear_data() {
  data_.Clear();
}
inline const ::scan_context_io::ScanContext& ScanContexts::data(int index) const {
  // @@protoc_insertion_point(field_get:scan_context_io.ScanContexts.data)
  return data_.Get(index);
}
inline ::scan_context_io::ScanContext* ScanContexts::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:scan_context_io.ScanContexts.data)
  return data_.Mutable(index);
}
inline ::scan_context_io::ScanContext* ScanContexts::add_data() {
  // @@protoc_insertion_point(field_add:scan_context_io.ScanContexts.data)
  return data_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::scan_context_io::ScanContext >*
ScanContexts::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:scan_context_io.ScanContexts.data)
  return &data_;
}
inline const ::google::protobuf::RepeatedPtrField< ::scan_context_io::ScanContext >&
ScanContexts::data() const {
  // @@protoc_insertion_point(field_list:scan_context_io.ScanContexts.data)
  return data_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace scan_context_io

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_scan_5fcontext_2fscan_5fcontexts_2eproto__INCLUDED