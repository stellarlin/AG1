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
#include <utility>
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
       : m_data(data), m_parent(std::move(parent)), m_left(nullptr), m_right(nullptr),
    m_left_count(0), m_right_count(0), m_left_height(0), m_right_height(0) {}

    size_t height () const {return std::max(m_left_height, m_right_height) + 1;}
    size_t count () const {return m_left_count + m_right_count + 1;}
    int get_balance() const{return  m_right_height - m_left_height;}
    size_t position(const pointer& root) const {return countNodes(0, root, true) - 1;}

    void updateHeightCount() {
      //UPDATE count
      m_left_count = m_left ? m_left->count() : 0;
      m_right_count = m_right ? m_right->count(): 0;

      //UPDATE height
      m_left_height = m_left ? m_left->height() : 0;
      m_right_height = m_right ? m_right-> height(): 0;
    }

    pointer update () {
      updateHeightCount();
      return rebalance();
    }

    pointer rebalance() {
      const int balance = get_balance();

      if (balance < 2 && balance > -2) return this->shared_from_this();
      //rotation is needed
 //LEFT
      if (balance > 0) {
        if (m_right->get_balance() < 0) m_right = m_right->rotate_right();  //LEFT RIGHT
        return rotate_left();
      }
      //RIGHT
      if (m_left->get_balance() > 0) m_left = m_left->rotate_left();  //LEFT RIGHT
      return rotate_right();
    }



    pointer rotate_left () {
      auto new_root = m_right;
      auto old_root = this->shared_from_this();
      new_root->m_parent = m_parent;

      //switch subtrees
      m_right = new_root->m_left;
      if(m_right) m_right->m_parent = old_root;

      //connect x and y
      new_root->m_left = old_root;
      m_parent = new_root;

      // Update heights and counts
      updateHeightCount();
      new_root->updateHeightCount();

      return new_root;
    }

    pointer rotate_right () {
      auto new_root = m_left;
      auto old_root = this->shared_from_this();
      new_root->m_parent = m_parent;

      //switch subtrees
      m_left = new_root->m_right;
      if(m_left) m_left->m_parent = old_root;

      //connect x and y
      new_root->m_right = this->shared_from_this();
      m_parent = new_root;


      // Update heights and counts
      updateHeightCount();
      new_root->updateHeightCount();

      return new_root;
    }



    pointer  push (const pointer&  added, size_t position) {
      pointer result = nullptr;
      // Determine whether to insert in the left or right subtree
      if (position <= m_left_count) {

        m_left = m_left ? m_left->push(added, position) :  added;
        m_left->m_parent = this->shared_from_this();
      } else {
       m_right  =  m_right ? m_right->push(added, position - m_left_count - 1) : added;
       m_right->m_parent = this->shared_from_this();
      }

      return update();
    }


    std::pair<pointer,size_t> pop (size_t position) {
      T deletedValue{};

      // Determine whether to insert in the left or right subtree
      if (position < m_left_count){

        auto  [newLeft, value] = m_left->pop(position);
        m_left = newLeft;
        deletedValue = value;
        m_left_count--;

      } else if (position  > m_left_count) {
       auto [newRight, value] =   m_right->pop( position- m_left_count - 1);
        m_right = newRight;
        deletedValue = value;
        m_right_count--;
      }
      else {

        if (!m_left && !m_right) {
          return {nullptr, m_data};
        }
        if (!m_left) {
          return {m_right, m_data};
        }
        if (!m_right) return {m_left, m_left->m_data};


        //# Case: Two children
        auto successor = m_right->findMin();

        //delete successor from right subtree
        auto [newRight, _] = m_right->pop(successor->position(m_right));

        newRight->m_parent = successor;
        successor->m_right = newRight;
        successor->m_left = m_left;
        successor->m_parent = m_parent;

        return {successor->update(), m_data};
      }


      return {update(),deletedValue};
    }

    pointer findMin() {
    return (!m_left) ? this->shared_from_this() : m_left->findMin();
    }

    size_t countNodes(size_t count,  const pointer& root, bool from_left) const
    {
      if (from_left) count += m_left_count + 1;
      return (root == this->shared_from_this()) ?  count
      : m_parent->countNodes(count, root, m_parent->m_right == this->shared_from_this());
   }

    T m_data;
    pointer m_parent;
    pointer m_left;
    pointer m_right;

    size_t m_left_count;
    size_t m_right_count;

    int m_left_height;
    int m_right_height;

  };

public:
  using pointer = std::shared_ptr<Node>;

  Queue() =default;
  bool empty() const { return m_root == nullptr;}
  size_t size() const {return m_size;}

  struct Ref {

    // Constructor that accepts a shared_ptr<Node<T>>
    explicit Ref(std::weak_ptr<Node> node = nullptr) : m_id(std::move(node)) {}

    // Copy constructor
    Ref(const Ref& other) : m_id(other.m_id) {}

    // Move constructor
    Ref(Ref&& other) noexcept : m_id(std::move(other.m_id)) {}

    void print() const {
      auto node = m_id.lock(); // Lock the weak_ptr to get a shared_ptr
      if (node) std::cout << node->m_data;
    }
  private:
      std::weak_ptr<Node> m_id; // Identifier managed by the Queue
    friend struct Queue; // Allows Queue to manage Ref instances
  };

  pointer findLast()
  {
    auto last = m_root;
    while (last->m_right != nullptr){last = last->m_right;}
    return last;
  }

  Ref push_last(T x) {
    pointer added = std::make_shared<Node>(std::move(x));

    if(empty()){
      m_root = added;
      ++m_size;
      return Ref(added);
    }

    m_root = m_root->push(added, m_size);
    ++m_size;
    return Ref(added);
  }

  size_t position(const Ref& it) const {
    auto node = it.m_id.lock(); // Lock the weak_ptr to get a shared_ptr
    if (!node) throw std::out_of_range("Invalid Ref"); // Check if the weak_ptr is expired


    if (node == nullptr) throw std::out_of_range("Empty queue");
    return node->position(m_root);
  }

  T pop_first() // throw std::out_of_range if empty
  {
    if (empty()) throw std::out_of_range("Empty queue");

    auto[new_root, res] = m_root->pop(0);
    m_root = new_root;
    m_size--;
    return res;
  }

  void jump_ahead(const Ref& it, size_t positions) {
    if (!positions) return;

    auto old_pos = position(it);
    auto node = it.m_id.lock();
    auto[new_root, _]= m_root->pop(old_pos);

    *node = Node(node->m_data);
    m_root = new_root->push(node,
      (old_pos < positions ? 0 : old_pos - positions));
  }

  // Print tree in a structured format (for visualization)
  void printNode(const pointer& node, const std::string &indent = "", bool isLeft = true) const{
    if (node) {
      std::cout << indent;
      std::cout << (isLeft ? "L-- " : "R-- ");
      std::cout << "#" << node->position(m_root) <<":("<< node->m_data  << " - from: "
      << (node->m_parent ? node->m_parent->position(m_root) : 0 )<<")"<< std::endl;

      printNode(node->m_left, indent + (isLeft ? "|   " : "    "), true);
      printNode(node->m_right, indent + (isLeft ? "|   " : "    "), false);
    }
  }

  void print() const {
    if (!m_root) {
      std::cout << "(empty tree)\n";
      return;
    }

    printNode(m_root);
    for (int x = 0; x < 100; ++x) std::cout<<"-";
      std::cout<<std::endl;
  }

 // private:
  pointer m_root = nullptr;
  size_t m_size = 0;
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

  for (const auto& x : refs) {
    x.print();
    std::cout << " ";
  }
  std::cout << std::endl;



  for (int i = 0; i < TOT; i++) CHECK(Q.position(refs[i]), i);

 //Q.print();

  Q.jump_ahead(refs[0], 15);

  Q.jump_ahead(refs[3], 0);


  CHECK(Q.size(), TOT);
  for (int i = 0; i < TOT; i++) {
    auto pos = Q.position(refs[i]);
    if ((int)pos != i) std::cout << "Ref pos: " << i << " - result pos: " << pos << std::endl;
    CHECK(Q.position(refs[i]), i);
  }

  Q.jump_ahead(refs[8], 100);
  Q.print();
  Q.jump_ahead(refs[9], 100);
  Q.print();
  Q.jump_ahead(refs[7], 1);
  Q.print();
/*
  static_assert(RUN == 10 && TOT >= 30);
  for (int i : { 9, 8, 0, 1, 2, 3, 4, 5}) {
    auto value = Q.pop_first();
    if (value != i) std::cout << "Ref value: " << i << " - result value: " << value << std::endl;
    CHECK(value, i);
  }
//Q.print();
  for (int i : { 7, 6 }) {
    auto value = Q.pop_first();
    if (value != i) std::cout << "Ref value: " << i << " - result value: " << value << std::endl;
    CHECK(value, i);
  }
  for (int i = 0; i < TOT*2 / 3; i++) {
    CHECK(Q.pop_first(), i % RUN);
    CHECK(Q.size(), TOT - 11 - i);
  }

  CHECK(Q.empty(), false);
  */
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

void test3() {
  Queue<int> q;

  for(int i = 1; i < 10; i++) {
    Queue<int>::Ref p = q.push_last(i);
    std::cout <<"Position:"<<q.position(p)<<std::endl;
    q.print();
  }
}

void test4() {
  Queue<int> q;

  for(int i = 1; i < 10; i++) {
    Queue<int>::Ref p = q.push_last(i);
  }
  q.print();

  std::cout <<"Position: "<<q.m_root->m_right->m_right->m_data<<","<<q.position(Queue<int>::Ref(q.m_root->m_right->m_right))<<std::endl;

  auto first = q.m_root;
  auto second =  q.m_root->m_right->m_right->m_right;

  std::cout <<"Position: ("<<first->m_data << " " << second->m_data<<"):"<<second->countNodes(0, first, true)<<std::endl;
  std::cout <<"Position: ("<<first->m_right->m_data << " " << second->m_data<<"):"<<second->countNodes(0, first->m_right, true)<<std::endl;
}

void test5() {
  Queue<int> q;

  for(int i = 1; i < 10; i++) {
    Queue<int>::Ref p = q.push_last(i);
  }
  while (!q.empty())
  {
    std::cout << "Value deleted:" << q.pop_first()<<std::endl;
    q.print();
  }
}

void test6() {
  Queue<int> q;

  for(int i = 1; i < 6; i++) {
    Queue<int>::Ref p = q.push_last(i);
  }
  Queue<int>::Ref p (q.m_root);
  q.print();
  q.jump_ahead(p,2);
  q.print();
}


int main() {
  int ok = 0, fail = 0;
 // test1(ok, fail);
  if (!fail) test2(ok, fail);
 // if (!fail) test_speed<1'000>(ok, fail);
 
  if (!fail) std::cout << "Passed all " << ok << " tests!" << std::endl;
  else std::cout << "Failed " << fail << " of " << (ok + fail) << " tests." << std::endl;
}

#endif


