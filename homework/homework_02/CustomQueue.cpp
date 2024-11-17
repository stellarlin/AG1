
#ifndef __PROGTEST__
#include <cassert>
#include <iomanip>
#include <cstdint>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <algorithm>
#include <bitset>
#include <list>
#include <array>
#include <vector>
#include <deque>
#include <unordered_set>
#include <unordered_map>
#include <stack>
#include <queue>
#include <random>
#include <type_traits>
#include <utility>

#endif

template < typename T >
struct Queue {
private:
  struct Node  : public std::enable_shared_from_this<Node>{
    using pointer = std::shared_ptr<Node>;

    explicit Node(T data, pointer parent = nullptr)
       : m_data(data), m_parent(parent), m_left(nullptr), m_right(nullptr),
    m_left_count(0), m_right_count(0), m_left_height(0), m_right_height(0) {}

    std::shared_ptr<Node> createLeft(T data) {
      m_left = std::make_shared<Node>(std::move(data), this->shared_from_this());
      m_left_count++;
      m_right_height++;
      return m_left;
    }

    std::shared_ptr<Node> createRight(T data) {
      m_right = std::make_shared<Node>(std::move(data), this->shared_from_this());
      m_right_count++;
      m_right_height++;
      return m_right;
    }

    size_t height () const {
      return std::max(m_left_height, m_right_height);
    }

    size_t count () const {
      return m_left_count + m_right_count + 1;
    }


    void update () {
      if (m_parent == nullptr)return;
      rebalance().m_parent->update();
    }

    Node & rebalance() {
      int balance = get_balance();

      if (balance < 2 && balance > -2) return *this;
      //rotation

      //LEFT
      if (balance > 0) {
        auto left_balance = m_right->get_balance();
        //LEFT
        if (left_balance > 0) return rotate_left ();
        //RIGHT
        else {
          m_right->rotate_right();
          return rotate_left();
        }
      }
      //RIGHT
      else {
        auto right_balance = m_left->get_balance();
        //LEFT
        if (right_balance < 0) return  rotate_right ();
        //RIGHT
        else {
          m_left->rotate_left();
          return rotate_right();
        }
      }
    }

     int get_balance() const{
      return  static_cast<int>(m_right_height - m_left_height);
    }

    Node & rotate_left () {
      auto z = m_parent;
      auto x = this->shared_from_this();
      auto y = m_left;
      y->m_parent = z;

      //switch subtrees
      x->m_left = y->m_right;
      x->m_left_count = y->m_right_count;
      x.m_left_height = y->m_right_height;

      //connect x and y
      y->m_right = x;
      x->m_parent = y;

      return *y;
      //update y
      y->m_right_height = x-height();
      y->m_right_count = x.count();
    }

     Node & rotate_right () {
      auto src = m_parent;
      auto x = this->shared_from_this();
      auto y = m_right;
      y->m_parent = src;

      //switch subtrees
      x->m_right = y->m_left;
      x->m_right_count = y->m_left_count;
      x.m_right_height = y->m_left_height;

      //connect x and y
      y->m_left = x;
      x->m_parent = y;

      return *y;

      //update y
      y->m_left_height = x-height();
      y->m_left_count = x.count();
    }

    std::shared_ptr<Node> push (T data, bool is_right) {
      //push new node
      auto added = is_right ? createRight(std::move(data))
        : createLeft(std::move(data));

      update();
      return added;
    }

    size_t countNodes(size_t count, bool from_left = true) {

      return count;
    }

    T m_data;
    pointer m_parent;
    pointer m_left;
    pointer m_right;

    size_t m_left_count;
    size_t m_right_count;

    size_t m_left_height;
    size_t m_right_height;

  };

public:
  Queue() : m_size(0){}
  bool empty() const { return m_root == nullptr;}
  size_t size() const {return m_size;}

  struct Ref {
  private:
    // Constructor that accepts a shared_ptr<Node<T>>
    explicit Ref(T data, std::shared_ptr<Node> parent = {}) {
      m_ref = std::make_shared<Node>(std::move(data), parent);
    }
    explicit Ref(std::shared_ptr<Node> node) : m_ref(node) {}

    // Copy constructor
    Ref(const Ref& other) : m_ref(other.m_ref) {}

    // Move constructor
    Ref(Ref&& other) noexcept : m_ref(std::move(other.m_ref)) {
      other.m_ref = nullptr;
    }
    // check if ref is empty
    bool empty() const {return m_ref == nullptr;}

    std::shared_ptr<Node> m_ref;
  };

  void init (T x) {
    m_root = Ref(x);
    m_last = m_root;
    ++m_size;
  }


  Ref push_last(T x) {
    if(empty()) init (std::move(x));
    else {
    m_last = m_last.push(std::move(x), true);
    ++m_size;
    }
    return m_last;
  }

  T pop_first(); // throw std::out_of_range if empty

  size_t position(const Ref& it) const {
    if (it.empty()) return 0;
    return it.m_ref->countNodes(0, true);
  }

  void jump_ahead(const Ref& it, size_t positions);

private:
  Ref  m_root;
  Ref m_last;
  size_t m_size;
};

#ifndef __PROGTEST__

////////////////// Dark magic, ignore ////////////////////////

template < typename T >
auto quote(const T& t) { return t; }

std::string quote(const std::string& s) {
  std::string ret = "\"";
  for (char c : s) if (c != '\n') ret += c; else ret += "\\n";
  return ret + "\"";
}

#define STR_(a) #a
#define STR(a) STR_(a)

#define CHECK_(a, b, a_str, b_str) do { \
    auto _a = (a); \
    decltype(a) _b = (b); \
    if (_a != _b) { \
      std::cout << "Line " << __LINE__ << ": Assertion " \
        << a_str << " == " << b_str << " failed!" \
        << " (lhs: " << quote(_a) << ")" << std::endl; \
      fail++; \
    } else ok++; \
  } while (0)

#define CHECK(a, b) CHECK_(a, b, #a, #b)

#define CHECK_EX(expr, ex) do { \
    try { \
      (expr); \
      fail++; \
      std::cout << "Line " << __LINE__ << ": Expected " STR(expr) \
        " to throw " #ex " but no exception was raised." << std::endl; \
    } catch (const ex&) { ok++; \
    } catch (...) { \
      fail++; \
      std::cout << "Line " << __LINE__ << ": Expected " STR(expr) \
        " to throw " #ex " but got different exception." << std::endl; \
    } \
  } while (0)
 
////////////////// End of dark magic ////////////////////////


void test1(int& ok, int& fail) {
  Queue<int> Q;
  CHECK(Q.empty(), true);
  CHECK(Q.size(), 0);

  constexpr int RUN = 10, TOT = 105;

  for (int i = 0; i < TOT; i++) {
    Q.push_last(i % RUN);
    CHECK(Q.empty(), false);
    CHECK(Q.size(), i + 1);
  }

  for (int i = 0; i < TOT; i++) {
    CHECK(Q.pop_first(), i % RUN);
    CHECK(Q.size(), TOT - 1 - i);
  }

  CHECK(Q.empty(), true);
}

void test2(int& ok, int& fail) {
  Queue<int> Q;
  CHECK(Q.empty(), true);
  CHECK(Q.size(), 0);
  std::vector<decltype(Q.push_last(0))> refs;

  constexpr int RUN = 10, TOT = 105;

  for (int i = 0; i < TOT; i++) {
    refs.push_back(Q.push_last(i % RUN));
    CHECK(Q.size(), i + 1);
  }
  
  for (int i = 0; i < TOT; i++) CHECK(Q.position(refs[i]), i);

  Q.jump_ahead(refs[0], 15);
  Q.jump_ahead(refs[3], 0);
  
  CHECK(Q.size(), TOT);
  for (int i = 0; i < TOT; i++) CHECK(Q.position(refs[i]), i);

  Q.jump_ahead(refs[8], 100);
  Q.jump_ahead(refs[9], 100);
  Q.jump_ahead(refs[7], 1);

  static_assert(RUN == 10 && TOT >= 30);
  for (int i : { 9, 8, 0, 1, 2, 3, 4, 5, 7, 6 })
    CHECK(Q.pop_first(), i);

  for (int i = 0; i < TOT*2 / 3; i++) {
    CHECK(Q.pop_first(), i % RUN);
    CHECK(Q.size(), TOT - 11 - i);
  }

  CHECK(Q.empty(), false);
}

template < int TOT >
void test_speed(int& ok, int& fail) {
  Queue<int> Q;
  CHECK(Q.empty(), true);
  CHECK(Q.size(), 0);
  std::vector<decltype(Q.push_last(0))> refs;

  for (int i = 0; i < TOT; i++) {
    refs.push_back(Q.push_last(i));
    CHECK(Q.size(), i + 1);
  }
  
  for (int i = 0; i < TOT; i++) {
    CHECK(Q.position(refs[i]), i);
    Q.jump_ahead(refs[i], i);
  }

  for (int i = 0; i < TOT; i++) CHECK(Q.position(refs[i]), TOT - 1 - i);

  for (int i = 0; i < TOT; i++) {
    CHECK(Q.pop_first(), TOT - 1 - i);
    CHECK(Q.size(), TOT - 1 - i);
  }

  CHECK(Q.empty(), true);
}

int main() {
  int ok = 0, fail = 0;
  test1(ok, fail);
  if (!fail) test2(ok, fail);
  if (!fail) test_speed<1'000>(ok, fail);
 
  if (!fail) std::cout << "Passed all " << ok << " tests!" << std::endl;
  else std::cout << "Failed " << fail << " of " << (ok + fail) << " tests." << std::endl;
}

#endif


