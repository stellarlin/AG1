
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
  struct Node  : public std::enable_shared_from_this<Node>{
    using pointer = std::shared_ptr<Node>;
    explicit Node(T data, pointer parent = nullptr)
       : m_data(data), m_parent(parent), m_left(nullptr), m_right(nullptr),
    m_left_count(0), m_right_count(0), m_left_height(0), m_right_height(0) {}



    size_t height () const {return std::max(m_left_height, m_right_height) + 1;}
    size_t count () const {return m_left_count + m_right_count + 1;}
    int get_balance() const{return  m_right_height - m_left_height;}


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
      pointer new_node;

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
      new_root->m_parent = m_parent;

      //switch subtrees
      m_right = new_root->m_left;

      //connect x and y
      new_root->m_left = this->shared_from_this();
      m_parent = new_root;

      // Update heights and counts
      updateHeightCount();
      new_root->updateHeightCount();

      return new_root->m_right;
    }

    pointer rotate_right () {
      auto new_root = m_left;
      new_root->m_parent = m_parent;

      //switch subtrees
      m_left = new_root->m_right;

      //connect x and y
      new_root->m_right = this->shared_from_this();
      m_parent = new_root;



      // Update heights and counts
      updateHeightCount();
      new_root->updateHeightCount();

      return new_root;
    }
    pointer createLeft(T data) {
      m_left = std::make_shared<Node>(std::move(data), this->shared_from_this());
      m_left_count++;
      return m_left;
    }

    pointer createRight(T data) {
      m_right = std::make_shared<Node>(std::move(data), this->shared_from_this());
      m_right_count++;
      return m_right;
    }

    pointer push (T data, bool is_right) {
      //push new node
      auto added = is_right ? createRight(std::move(data))
        : createLeft(std::move(data));

      return added;
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

  Queue() : m_root(nullptr), m_size(0){}
  bool empty() const { return m_root == nullptr;}
  size_t size() const {return m_size;}

  struct Ref {

    // Constructor that accepts a shared_ptr<Node<T>>
    explicit Ref(pointer node) : m_ref(node) {}

    // Copy constructor
    Ref(const Ref& other) : m_ref(other.m_ref) {}

    // Move constructor
    Ref(Ref&& other) noexcept : m_ref(std::move(other.m_ref)) {
      other.m_ref = nullptr;
    }
  private:    // check if ref is empty
    bool empty() const {return m_ref == nullptr;}

    pointer m_ref;
  };

  pointer findLast()
  {
   auto last = m_root;
   while (last->m_right != nullptr){last = last->m_right;}
   return last;
  }

  Ref push_last(T x) {
    if(empty()){
      m_root = std::make_shared<Node>(std::move(x));
      ++m_size;
      return Ref(m_root);
    }
    auto last = findLast();
    last = last->push(std::move(x), true);

    pointer current = last;
    auto new_subtree = nullptr;
    while (current) {
      if (!current->m_parent) {
        m_root = current->update();
        break;
      }
      current = current->update();
      ;
    }
    ++m_size;
    return Ref(last);
  }

/*
  T pop_first(); // throw std::out_of_range if empty

  size_t position(const Ref& it) const {
 //   if (it.empty()) return 0;
 //   return it.m_ref->countNodes(0, true);
  }

  void jump_ahead(const Ref& it, size_t positions);
*/

  // Print tree in a structured format (for visualization)
  void printNode(pointer node, std::string indent = "", bool isLeft = true) const{
    if (node) {
      std::cout << indent;
      std::cout << (isLeft ? "L-- " : "R-- ");
      std::cout << node->m_data << std::endl;

      printNode(node->m_left, indent + (isLeft ? "|   " : "    "), true);
      printNode(node->m_right, indent + (isLeft ? "|   " : "    "), false);
    }
  }

  void print() {
    if (!m_root) {
      std::cout << "(empty tree)\n";
      return;
    }

   printNode(m_root);
  }

private:
  pointer m_root;
  size_t m_size;
};

#ifndef __PROGTEST__

int main() {
Queue<int> q;
  q.push_last(1);
  q.push_last(2);
  q.push_last(3);
 // q.push_last(4);
 // q.push_last(5);
  q.print();
}
#endif


