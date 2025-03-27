/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <deque>
#include <utility>

/**
 * @brief StampedDeque is a a deque which for every value also stores a secondary
 * index parameter.
 *
 * @tparam  T  type of values to store
 * @tparam  C  type of secondary index parameter
 */
template <typename T, typename C>
using StampedDeque = std::deque<std::pair<C, T>>;

/**
 * @brief Ring-buffer-like container for keeping only values within a fixed
 * range of a secondary index parameter.
 *
 * A DifferenceBasedBuffer is a special type of deque similar to a
 * fixed length circular buffer. Instead of having a fixed length, the
 * DifferenceBasedBuffer only keeps those values, whose secondary stamp value is
 * within a fixed range of the most recently inserted value's stamp. As an
 * example, a DifferenceBasedBuffer can thus be used for keeping all values with
 * a stamp within the last 5 minutes by using timestamps as the secondary stamp
 * argument upon insertion.
 *
 * @tparam  T  type of values to store
 * @tparam  C  type of secondary index parameter
 * @tparam  D  type of secondary index parameter range (C-C)
 */
template <typename T, typename C = int, typename D = C>
class DifferenceBasedBuffer : public StampedDeque<T, C> {
 public:
  /**
   * @brief Constructs a new DifferenceBasedBuffer object.
   *
   * @param  diff  fixed range from most recent stamp within which values are
   * kept in the buffer
   */
  DifferenceBasedBuffer(const D& diff = D(0, 0));

  /**
   * @brief Sets new difference. Will affect subsequent calls to update().
   */
  void setDifference(const D& diff);

  /**
   * @brief Adds new element to the end.
   *
   * All elements with stamps outside `(stamp - diff_, stamp]` will be removed.
   *
   * @param  val    new value to add
   * @param  stamp  stamp of new element
   */
  void push_back(const T& val, const C& stamp = C());

  /**
   * @brief Adds new element to the front.
   *
   * All elements with stamps outside `(stamp - diff_, stamp]` will be removed.
   *
   * @param  val    new value to add
   * @param  stamp  stamp of new element
   */
  void push_front(const T& val, const C& stamp = C());

  /**
   * @brief Adds new element at position.
   *
   * All elements with stamps outside `(stamp - diff_, stamp]` will be removed.
   *
   * @param  position  iterator before which the element will be inserted
   * @param  val       new value to add
   * @param  stamp     stamp of new element
   */
  void insert(typename StampedDeque<T, C>::const_iterator position, const T& val, const C& stamp = C());

  /**
   * @brief Accesses specified element.
   *
   * @param  n  index of requested element
   *
   * @return  T&  reference to requested element
   */
  T& operator[](const int n);

  /**
   * @brief Accesses specified element.
   *
   * @param  n  index of requested element
   *
   * @return  const T&  reference to requested element
   */
  const T& operator[](const int n) const;

  /**
   * @brief Accesses specified element.
   *
   * @param  n  index of requested element
   *
   * @return  T&  reference to requested element
   */
  T& at(const int n);

  /**
   * @brief Accesses specified element.
   *
   * @param  n  index of requested element
   *
   * @return  const T&  reference to requested element
   */
  const T& at(const int n) const;

  /**
   * @brief Accesses specified element including stamp.
   *
   * @param   n  index of requested element
   *
   * @return  std::pair<C, T>&  reference to requested pair
   */
  std::pair<C, T>& get(const int n);

  /**
   * @brief Accesses specified element including stamp.
   *
   * @param   n  index of requested element
   *
   * @return  const std::pair<C, T>&  reference to requested pair
   */
  const std::pair<C, T>& get(const int n) const;

  /**
   * @brief Accesses first element.
   *
   * @return  T&  reference to first element
   */
  T& front();

  /**
   * @brief Accesses first element.
   *
   * @return  const T&  reference to first element
   */
  const T& front() const;

  /**
   * @brief Accesses last element.
   *
   * @return  T&  reference to last element
   */
  T& back();

  /**
   * @brief Accesses last element.
   *
   * @return  const T&  reference to last element
   */
  const T& back() const;

 protected:
  /**
   * @brief Removes all elements with stamps outside of (stamp - diff_, stamp].
   *
   * @param  stamp  upper bound of range within which to keep values
   */
  void update(const C& stamp);

  /**
   * @brief Difference value
   *
   * Used for removing elements when the difference between their stamp value
   * the newest stamp value is greater than this difference value.
   */
  D diff_;
};

#include "event_detector/DifferenceBasedBuffer.tpp"