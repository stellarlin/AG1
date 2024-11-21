
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
    size_t position(pointer root) const {return countNodes(0, root, true) - 1;}

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



    pointer push (pointer node, size_t position) {

      // Determine whether to insert in the left or right subtree
      if (position <= m_left_count) {
        m_left = m_left ? m_left->push(node, position) : node;
        m_left->m_parent = this->shared_from_this();
      } else {
       m_right =  m_right? m_right->push(node, position) : node;
       m_right->m_parent = this->shared_from_this();
      }

      return update();
    }


    std::pair<pointer,size_t> pop (size_t position) {
      T deletedValue;

      // Determine whether to insert in the left or right subtree
      if (position <= m_left_count){
        auto  [newLeft, value] = m_left->pop(position);
        m_left = newLeft;
        deletedValue = value;
        m_left_count--;

      } else if (position > m_left_count + 1) {
       auto [newRight, value] =   m_right->pop( position);
        m_right = newRight;
        deletedValue = value;
        m_right_count--;
      }
      else {
        deletedValue = m_data;

        if (!m_left && !m_right) {
          return {nullptr, deletedValue};
        }
        if (!m_left) {
          return {m_right, deletedValue};
        }
        if (!m_right) return {m_left, deletedValue};


        //# Case: Two children
        auto successor = m_right->findMin();
        m_data = successor->m_data;
        auto [newRight, _] = m_right->pop(successor->position(m_right) + 1);
        m_right = newRight;
        m_right_count--;
      }


      return {update(),deletedValue};
    }

    pointer findMin() {
    return (!m_left) ? this->shared_from_this() : m_left->findMin();
    }

    size_t countNodes(size_t count,  pointer root, bool from_left) const
    {
      if (from_left) count += m_left_count + 1;
      return (root == this->shared_from_this()) ?  count : m_parent->countNodes(count, root, m_parent->m_right == this->shared_from_this());
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
    explicit Ref(pointer node = nullptr) : m_id(node) {}

    // Copy constructor
    Ref(const Ref& other) : m_id(other.m_id) {}

    // Move constructor
    Ref(Ref&& other) noexcept : m_id(std::move(other.m_id)) {
      other.m_id = nullptr;
    }
  private:
    pointer m_id; // Identifier managed by the Queue
    friend class Queue; // Allows Queue to manage Ref instances
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
    auto added = std::make_shared<Node>(std::move(x));
     m_root = m_root->push(added, m_size);
    ++m_size;
    return Ref(added);
  }


  size_t position(const Ref& it) const {
    if (it.m_id == nullptr) return -1;
    return it.m_id->position(m_root);
  }


  T pop_first() // throw std::out_of_range if empty
  {
    if (empty()) throw std::out_of_range("Empty queue");

    auto[new_root, res] = m_root->pop(1);
    m_root = new_root;
    return res;
  }



  void jump_ahead(const Ref& it, size_t positions) {

    auto old_pos = position(it)+1;;
    auto[new_root, old_val]= m_root->pop(old_pos);
    pointer new_node = std::make_shared<Node>(std::move(old_val));
     m_root= new_root->push(new_node, (old_pos <= positions ? 1 : old_pos - positions));

  }


  // Print tree in a structured format (for visualization)
  void printNode(pointer node, std::string indent = "", bool isLeft = true) const{
    if (node) {
      std::cout << indent;
      std::cout << (isLeft ? "L-- " : "R-- ");
      std::cout << node->m_data  << std::endl;

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

//private:
  pointer m_root;
  size_t m_size;
};

#ifndef __PROGTEST__

void test1() {
  Queue<int> q;

  for(int i = 1; i < 10; i++) {
     Queue<int>::Ref p = q.push_last(i);
    std::cout <<"Position:"<<q.position(p)<<std::endl;
    q.print();
  }
}

void test2() {
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

void test3() {
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

void test4() {
  Queue<int> q;

  for(int i = 1; i < 10; i++) {
    Queue<int>::Ref p = q.push_last(i);
  }
  Queue<int>::Ref p (q.m_root);
  q.jump_ahead(p,2);
  q.print();
}

int main() {
test4();
}
#endif


