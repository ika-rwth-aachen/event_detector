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

template <typename T, typename C, typename D>
DifferenceBasedBuffer<T, C, D>::DifferenceBasedBuffer(const D& diff) : StampedDeque<T, C>(), diff_(diff) {}

template <typename T, typename C, typename D>
void DifferenceBasedBuffer<T, C, D>::setDifference(const D& diff) {
  diff_ = diff;
}

template <typename T, typename C, typename D>
void DifferenceBasedBuffer<T, C, D>::push_back(const T& val, const C& stamp) {
  StampedDeque<T, C>::emplace_back(stamp, val);
  this->update(stamp);
}

template <typename T, typename C, typename D>
void DifferenceBasedBuffer<T, C, D>::push_front(const T& val, const C& stamp) {
  StampedDeque<T, C>::emplace_front(stamp, val);
  this->update(stamp);
}

template <typename T, typename C, typename D>
void DifferenceBasedBuffer<T, C, D>::insert(typename StampedDeque<T, C>::const_iterator position, const T& val,
                                            const C& stamp) {
  auto mutable_position = this->begin() + std::distance(this->cbegin(), position);
  StampedDeque<T, C>::emplace(mutable_position, stamp, val);
  this->update(stamp);
}

template <typename T, typename C, typename D>
T& DifferenceBasedBuffer<T, C, D>::operator[](int n) {
  return StampedDeque<T, C>::operator[](n).second;
}

template <typename T, typename C, typename D>
const T& DifferenceBasedBuffer<T, C, D>::operator[](int n) const {
  return StampedDeque<T, C>::operator[](n).second;
}

template <typename T, typename C, typename D>
T& DifferenceBasedBuffer<T, C, D>::at(const int n) {
  return StampedDeque<T, C>::at(n).second;
}

template <typename T, typename C, typename D>
const T& DifferenceBasedBuffer<T, C, D>::at(const int n) const {
  return StampedDeque<T, C>::at(n).second;
}

template <typename T, typename C, typename D>
std::pair<C, T>& DifferenceBasedBuffer<T, C, D>::get(const int n) {
  return StampedDeque<T, C>::at(n);
}

template <typename T, typename C, typename D>
const std::pair<C, T>& DifferenceBasedBuffer<T, C, D>::get(const int n) const {
  return StampedDeque<T, C>::at(n);
}

template <typename T, typename C, typename D>
T& DifferenceBasedBuffer<T, C, D>::front() {
  return StampedDeque<T, C>::front().second;
}

template <typename T, typename C, typename D>
T& DifferenceBasedBuffer<T, C, D>::back() {
  return StampedDeque<T, C>::back().second;
}

template <typename T, typename C, typename D>
void DifferenceBasedBuffer<T, C, D>::update(const C& stamp) {
  if (diff_ <= D(0, 0)) {  // no limit
    return;
  }
  for (auto it = this->cbegin(); it != this->cend();) {
    if ((stamp - it->first) >= diff_)
      it = this->erase(it);
    else
      it++;
  }
}