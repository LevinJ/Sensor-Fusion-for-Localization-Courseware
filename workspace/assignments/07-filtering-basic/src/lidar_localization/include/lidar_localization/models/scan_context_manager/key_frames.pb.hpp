// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: scan_context/key_frames.proto

#ifndef PROTOBUF_scan_5fcontext_2fkey_5fframes_2eproto__INCLUDED
#define PROTOBUF_scan_5fcontext_2fkey_5fframes_2eproto__INCLUDED

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
void protobuf_AddDesc_scan_5fcontext_2fkey_5fframes_2eproto();
void protobuf_AssignDesc_scan_5fcontext_2fkey_5fframes_2eproto();
void protobuf_ShutdownFile_scan_5fcontext_2fkey_5fframes_2eproto();

class KeyFrame;
class KeyFrames;
class Quat;
class Trans;

// ===================================================================

class Trans : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.Trans) */ {
 public:
  Trans();
  virtual ~Trans();

  Trans(const Trans& from);

  inline Trans& operator=(const Trans& from) {
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
  static const Trans& default_instance();

  void Swap(Trans* other);

  // implements Message ----------------------------------------------

  inline Trans* New() const { return New(NULL); }

  Trans* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Trans& from);
  void MergeFrom(const Trans& from);
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
  void InternalSwap(Trans* other);
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

  // required float x = 1;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  float x() const;
  void set_x(float value);

  // required float y = 2;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  float y() const;
  void set_y(float value);

  // required float z = 3;
  bool has_z() const;
  void clear_z();
  static const int kZFieldNumber = 3;
  float z() const;
  void set_z(float value);

  // @@protoc_insertion_point(class_scope:scan_context_io.Trans)
 private:
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();
  inline void set_has_z();
  inline void clear_has_z();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  float x_;
  float y_;
  float z_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fkey_5fframes_2eproto();

  void InitAsDefaultInstance();
  static Trans* default_instance_;
};
// -------------------------------------------------------------------

class Quat : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.Quat) */ {
 public:
  Quat();
  virtual ~Quat();

  Quat(const Quat& from);

  inline Quat& operator=(const Quat& from) {
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
  static const Quat& default_instance();

  void Swap(Quat* other);

  // implements Message ----------------------------------------------

  inline Quat* New() const { return New(NULL); }

  Quat* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Quat& from);
  void MergeFrom(const Quat& from);
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
  void InternalSwap(Quat* other);
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

  // required float w = 1;
  bool has_w() const;
  void clear_w();
  static const int kWFieldNumber = 1;
  float w() const;
  void set_w(float value);

  // required float x = 2;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 2;
  float x() const;
  void set_x(float value);

  // required float y = 3;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 3;
  float y() const;
  void set_y(float value);

  // required float z = 4;
  bool has_z() const;
  void clear_z();
  static const int kZFieldNumber = 4;
  float z() const;
  void set_z(float value);

  // @@protoc_insertion_point(class_scope:scan_context_io.Quat)
 private:
  inline void set_has_w();
  inline void clear_has_w();
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();
  inline void set_has_z();
  inline void clear_has_z();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  float w_;
  float x_;
  float y_;
  float z_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fkey_5fframes_2eproto();

  void InitAsDefaultInstance();
  static Quat* default_instance_;
};
// -------------------------------------------------------------------

class KeyFrame : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.KeyFrame) */ {
 public:
  KeyFrame();
  virtual ~KeyFrame();

  KeyFrame(const KeyFrame& from);

  inline KeyFrame& operator=(const KeyFrame& from) {
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
  static const KeyFrame& default_instance();

  void Swap(KeyFrame* other);

  // implements Message ----------------------------------------------

  inline KeyFrame* New() const { return New(NULL); }

  KeyFrame* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const KeyFrame& from);
  void MergeFrom(const KeyFrame& from);
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
  void InternalSwap(KeyFrame* other);
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

  // required .scan_context_io.Quat q = 1;
  bool has_q() const;
  void clear_q();
  static const int kQFieldNumber = 1;
  const ::scan_context_io::Quat& q() const;
  ::scan_context_io::Quat* mutable_q();
  ::scan_context_io::Quat* release_q();
  void set_allocated_q(::scan_context_io::Quat* q);

  // required .scan_context_io.Trans t = 2;
  bool has_t() const;
  void clear_t();
  static const int kTFieldNumber = 2;
  const ::scan_context_io::Trans& t() const;
  ::scan_context_io::Trans* mutable_t();
  ::scan_context_io::Trans* release_t();
  void set_allocated_t(::scan_context_io::Trans* t);

  // @@protoc_insertion_point(class_scope:scan_context_io.KeyFrame)
 private:
  inline void set_has_q();
  inline void clear_has_q();
  inline void set_has_t();
  inline void clear_has_t();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::scan_context_io::Quat* q_;
  ::scan_context_io::Trans* t_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fkey_5fframes_2eproto();

  void InitAsDefaultInstance();
  static KeyFrame* default_instance_;
};
// -------------------------------------------------------------------

class KeyFrames : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:scan_context_io.KeyFrames) */ {
 public:
  KeyFrames();
  virtual ~KeyFrames();

  KeyFrames(const KeyFrames& from);

  inline KeyFrames& operator=(const KeyFrames& from) {
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
  static const KeyFrames& default_instance();

  void Swap(KeyFrames* other);

  // implements Message ----------------------------------------------

  inline KeyFrames* New() const { return New(NULL); }

  KeyFrames* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const KeyFrames& from);
  void MergeFrom(const KeyFrames& from);
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
  void InternalSwap(KeyFrames* other);
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

  // repeated .scan_context_io.KeyFrame data = 1;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 1;
  const ::scan_context_io::KeyFrame& data(int index) const;
  ::scan_context_io::KeyFrame* mutable_data(int index);
  ::scan_context_io::KeyFrame* add_data();
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::KeyFrame >*
      mutable_data();
  const ::google::protobuf::RepeatedPtrField< ::scan_context_io::KeyFrame >&
      data() const;

  // @@protoc_insertion_point(class_scope:scan_context_io.KeyFrames)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::scan_context_io::KeyFrame > data_;
  friend void  protobuf_AddDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_AssignDesc_scan_5fcontext_2fkey_5fframes_2eproto();
  friend void protobuf_ShutdownFile_scan_5fcontext_2fkey_5fframes_2eproto();

  void InitAsDefaultInstance();
  static KeyFrames* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Trans

// required float x = 1;
inline bool Trans::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Trans::set_has_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Trans::clear_has_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Trans::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline float Trans::x() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Trans.x)
  return x_;
}
inline void Trans::set_x(float value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Trans.x)
}

// required float y = 2;
inline bool Trans::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Trans::set_has_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Trans::clear_has_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Trans::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline float Trans::y() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Trans.y)
  return y_;
}
inline void Trans::set_y(float value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Trans.y)
}

// required float z = 3;
inline bool Trans::has_z() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Trans::set_has_z() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Trans::clear_has_z() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Trans::clear_z() {
  z_ = 0;
  clear_has_z();
}
inline float Trans::z() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Trans.z)
  return z_;
}
inline void Trans::set_z(float value) {
  set_has_z();
  z_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Trans.z)
}

// -------------------------------------------------------------------

// Quat

// required float w = 1;
inline bool Quat::has_w() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Quat::set_has_w() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Quat::clear_has_w() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Quat::clear_w() {
  w_ = 0;
  clear_has_w();
}
inline float Quat::w() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Quat.w)
  return w_;
}
inline void Quat::set_w(float value) {
  set_has_w();
  w_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Quat.w)
}

// required float x = 2;
inline bool Quat::has_x() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Quat::set_has_x() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Quat::clear_has_x() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Quat::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline float Quat::x() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Quat.x)
  return x_;
}
inline void Quat::set_x(float value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Quat.x)
}

// required float y = 3;
inline bool Quat::has_y() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Quat::set_has_y() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Quat::clear_has_y() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Quat::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline float Quat::y() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Quat.y)
  return y_;
}
inline void Quat::set_y(float value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Quat.y)
}

// required float z = 4;
inline bool Quat::has_z() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Quat::set_has_z() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Quat::clear_has_z() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Quat::clear_z() {
  z_ = 0;
  clear_has_z();
}
inline float Quat::z() const {
  // @@protoc_insertion_point(field_get:scan_context_io.Quat.z)
  return z_;
}
inline void Quat::set_z(float value) {
  set_has_z();
  z_ = value;
  // @@protoc_insertion_point(field_set:scan_context_io.Quat.z)
}

// -------------------------------------------------------------------

// KeyFrame

// required .scan_context_io.Quat q = 1;
inline bool KeyFrame::has_q() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void KeyFrame::set_has_q() {
  _has_bits_[0] |= 0x00000001u;
}
inline void KeyFrame::clear_has_q() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void KeyFrame::clear_q() {
  if (q_ != NULL) q_->::scan_context_io::Quat::Clear();
  clear_has_q();
}
inline const ::scan_context_io::Quat& KeyFrame::q() const {
  // @@protoc_insertion_point(field_get:scan_context_io.KeyFrame.q)
  return q_ != NULL ? *q_ : *default_instance_->q_;
}
inline ::scan_context_io::Quat* KeyFrame::mutable_q() {
  set_has_q();
  if (q_ == NULL) {
    q_ = new ::scan_context_io::Quat;
  }
  // @@protoc_insertion_point(field_mutable:scan_context_io.KeyFrame.q)
  return q_;
}
inline ::scan_context_io::Quat* KeyFrame::release_q() {
  // @@protoc_insertion_point(field_release:scan_context_io.KeyFrame.q)
  clear_has_q();
  ::scan_context_io::Quat* temp = q_;
  q_ = NULL;
  return temp;
}
inline void KeyFrame::set_allocated_q(::scan_context_io::Quat* q) {
  delete q_;
  q_ = q;
  if (q) {
    set_has_q();
  } else {
    clear_has_q();
  }
  // @@protoc_insertion_point(field_set_allocated:scan_context_io.KeyFrame.q)
}

// required .scan_context_io.Trans t = 2;
inline bool KeyFrame::has_t() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void KeyFrame::set_has_t() {
  _has_bits_[0] |= 0x00000002u;
}
inline void KeyFrame::clear_has_t() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void KeyFrame::clear_t() {
  if (t_ != NULL) t_->::scan_context_io::Trans::Clear();
  clear_has_t();
}
inline const ::scan_context_io::Trans& KeyFrame::t() const {
  // @@protoc_insertion_point(field_get:scan_context_io.KeyFrame.t)
  return t_ != NULL ? *t_ : *default_instance_->t_;
}
inline ::scan_context_io::Trans* KeyFrame::mutable_t() {
  set_has_t();
  if (t_ == NULL) {
    t_ = new ::scan_context_io::Trans;
  }
  // @@protoc_insertion_point(field_mutable:scan_context_io.KeyFrame.t)
  return t_;
}
inline ::scan_context_io::Trans* KeyFrame::release_t() {
  // @@protoc_insertion_point(field_release:scan_context_io.KeyFrame.t)
  clear_has_t();
  ::scan_context_io::Trans* temp = t_;
  t_ = NULL;
  return temp;
}
inline void KeyFrame::set_allocated_t(::scan_context_io::Trans* t) {
  delete t_;
  t_ = t;
  if (t) {
    set_has_t();
  } else {
    clear_has_t();
  }
  // @@protoc_insertion_point(field_set_allocated:scan_context_io.KeyFrame.t)
}

// -------------------------------------------------------------------

// KeyFrames

// repeated .scan_context_io.KeyFrame data = 1;
inline int KeyFrames::data_size() const {
  return data_.size();
}
inline void KeyFrames::clear_data() {
  data_.Clear();
}
inline const ::scan_context_io::KeyFrame& KeyFrames::data(int index) const {
  // @@protoc_insertion_point(field_get:scan_context_io.KeyFrames.data)
  return data_.Get(index);
}
inline ::scan_context_io::KeyFrame* KeyFrames::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:scan_context_io.KeyFrames.data)
  return data_.Mutable(index);
}
inline ::scan_context_io::KeyFrame* KeyFrames::add_data() {
  // @@protoc_insertion_point(field_add:scan_context_io.KeyFrames.data)
  return data_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::scan_context_io::KeyFrame >*
KeyFrames::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:scan_context_io.KeyFrames.data)
  return &data_;
}
inline const ::google::protobuf::RepeatedPtrField< ::scan_context_io::KeyFrame >&
KeyFrames::data() const {
  // @@protoc_insertion_point(field_list:scan_context_io.KeyFrames.data)
  return data_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace scan_context_io

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_scan_5fcontext_2fkey_5fframes_2eproto__INCLUDED