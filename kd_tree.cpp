#ifndef LIDAR_LOCALIZATION_TOOLS_KD_TREE_
#define LIDAR_LOCALIZATION_TOOLS_KD_TREE_

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <queue>
#include <stack>
#include <vector>

#include "math.h"

namespace lidar_localization {

template <class T>
class KDNode {
 public:
  typedef std::shared_ptr<KDNode<T>> KDNodePtr;
  KDNode() = default;
  ~KDNode() = default;

  void SetParent(const KDNodePtr node_ptr) { parent_ = node_ptr; }
  void SetLeftChild(const KDNodePtr node_ptr) { left_child_ = node_ptr; }
  void SetRightChild(const KDNodePtr node_ptr) { right_child_ = node_ptr; }
  void SetData(const T& data) { data_ = data; }
  void SetSplit(const int split) { split_ = split; }
  void SetDist(const double dist) { dist_ = dist; }

  bool HasParent() const { return parent_ != nullptr; }
  bool HasLeftChild() const { return left_child_ != nullptr; }
  bool HasRightChild() const { return right_child_ != nullptr; }

  bool IsLeftChild(const KDNodePtr node_ptr) { return node_ptr->GetData()[split_] < data_[split_]; }

  KDNodePtr GetParent() const { return parent_; }
  KDNodePtr GetLeftChild() const { return left_child_; }
  KDNodePtr GetRightChild() const { return right_child_; }
  int GetSplit() const { return split_; }
  T GetData() const { return data_; }
  double GetDist() const { return dist_; }

 private:
  int split_ = 0;
  KDNodePtr parent_ = nullptr;
  KDNodePtr left_child_ = nullptr;
  KDNodePtr right_child_ = nullptr;
  T data_;
  double dist_;
};

template <class T>
class KDTree {
 public:
  typedef std::shared_ptr<KDNode<T>> KDNodePtr;

  KDTree() = default;
  ~KDTree() = default;

  void AddData(const T& element, const int dimension) {
    elements_.emplace_back(element);
    dimension_ = dimension;
  }
  void ClearData() { elements_.clear(); }
  bool HasData() const { return !elements_.empty(); }
  size_t DataSize() const { return elements_.size(); }

  bool GenerateTree() {
    if (!HasData() || dimension_ <= 0) {
      return false;
    }
    root_ = CreateNode(0, DataSize() - 1);
    return root_ != nullptr;
  }

  bool TreeValid() const { return root_ != nullptr; }

  KDNodePtr CreateNode(const size_t lower_idx, const size_t upper_idx) {
    KDNodePtr node_ptr(new KDNode<T>);
    if (lower_idx == upper_idx) {
      node_ptr->SetData(elements_[lower_idx]);
      std::cout << "lower_idx: " << lower_idx << " upper_idx: " << upper_idx << std::endl;
      PrintElement(node_ptr->GetData());
      return node_ptr;
    }
    // determine split with largest variation
    int split = 0;
    double max_var = 0.0;
    for (size_t i = 0; i < dimension_; ++i) {
      double var = CalculateVariation(i, lower_idx, upper_idx);
      if (var >= max_var) {
        max_var = var;
        split = i;
      }
    }
    std::cout << "max_var: " << max_var << " split: " << split << std::endl;
    node_ptr->SetSplit(split);
    // sort data set by split
    // and set mid value as node data
    InsertSort(split, lower_idx, upper_idx);
    size_t mid_idx = (lower_idx + upper_idx + 1ul) / 2ul;
    node_ptr->SetData(elements_[mid_idx]);
    std::cout << "lower_idx: " << lower_idx << " upper_idx: " << upper_idx << " mid_idx: " << mid_idx << std::endl;
    PrintElement(node_ptr->GetData());
    // get left & right node
    if (mid_idx > lower_idx) {
      KDNodePtr left_node_ptr = CreateNode(lower_idx, mid_idx - 1);
      left_node_ptr->SetParent(node_ptr);
      node_ptr->SetLeftChild(left_node_ptr);
    }
    if (mid_idx < upper_idx) {
      KDNodePtr right_node_ptr = CreateNode(mid_idx + 1, upper_idx);
      right_node_ptr->SetParent(node_ptr);
      node_ptr->SetRightChild(right_node_ptr);
    }

    return node_ptr;
  }

  bool FindNearestPoint(const T& target_point, T& nearest_point) {
    if (!HasData() || !TreeValid()) {
      return false;
    }

    auto func_is_leaf = [](KDNodePtr node_ptr) { return !(node_ptr->HasLeftChild() || node_ptr->HasRightChild()); };

    auto func_cal_dist_sqr = [this](KDNodePtr node_ptr0, KDNodePtr node_ptr1) {
      double sqr_sum = 0.0;
      for (int i = 0; i < this->dimension_; ++i) {
        sqr_sum += std::pow(node_ptr0->GetData()[i] - node_ptr1->GetData()[i], 2);
      }
      return sqr_sum;
    };

    auto func_judge_hypersurface_cut = [](KDNodePtr current_node_ptr, KDNodePtr target_node_ptr,
                                          const double radius_sqr) {
      auto current_split = current_node_ptr->GetSplit();
      double dist = std::fabs(current_node_ptr->GetData()[current_split] - target_node_ptr->GetData()[current_split]);
      // std::cout << "cur split: " << current_split << " cut dist: " << dist << " radius_sqr: " << radius_sqr <<
      // std::endl;
      return dist * dist <= radius_sqr;
    };

    // binary search & record search path
    KDNodePtr current_node_ptr = root_;
    std::stack<KDNodePtr> node_stack;
    node_stack.push(current_node_ptr);
    while (!func_is_leaf(current_node_ptr)) {
      auto current_node_data = current_node_ptr->GetData();
      auto current_split = current_node_ptr->GetSplit();
      if (current_node_data[current_split] > target_point[current_split] && current_node_ptr->HasLeftChild()) {
        current_node_ptr = current_node_ptr->GetLeftChild();
        node_stack.push(current_node_ptr);
      } else if (current_node_ptr->HasRightChild()) {
        current_node_ptr = current_node_ptr->GetRightChild();
        node_stack.push(current_node_ptr);
      }
    }

    // reverse search to update possible closer node
    KDNodePtr target_node_ptr(new KDNode<T>);
    target_node_ptr->SetData(target_point);
    KDNodePtr nearest_node_ptr = node_stack.top();
    // std::cout << "nearest node 0: ";
    // PrintElement(nearest_node_ptr->GetData());
    node_stack.pop();
    double min_dist_sqr = func_cal_dist_sqr(target_node_ptr, nearest_node_ptr);
    // std::cout << "min_dist_sqr 0: " << min_dist_sqr << std::endl;
    while (!node_stack.empty()) {
      current_node_ptr = node_stack.top();
      node_stack.pop();
      double cur_dist_sqr = func_cal_dist_sqr(target_node_ptr, current_node_ptr);
      // std::cout << "current node: ";
      // PrintElement(current_node_ptr->GetData());
      // std::cout << "cur_dist_sqr: " << cur_dist_sqr << std::endl;
      if (cur_dist_sqr < min_dist_sqr) {
        nearest_node_ptr = current_node_ptr;
        min_dist_sqr = cur_dist_sqr;
        // std::cout << "min_dist_sqr: " << min_dist_sqr << std::endl;
      }
      // std::cout << "current is leaf: " << func_is_leaf(current_node_ptr) << std::endl;
      if (!func_is_leaf(current_node_ptr)) {
        // if node doesn't intersect with target node's hyper surface
        // reverse search is done(if it does, there's a chance the other side
        // has closer dist node)
        if (func_judge_hypersurface_cut(current_node_ptr, target_node_ptr, min_dist_sqr)) {
          // std::cout << "cut with ";
          // PrintElement(current_node_ptr->GetData());
          if (current_node_ptr->IsLeftChild(target_node_ptr) && current_node_ptr->HasRightChild()) {
            current_node_ptr = current_node_ptr->GetRightChild();
            node_stack.push(current_node_ptr);
            // std::cout << "go to right child: ";
            // PrintElement(current_node_ptr->GetData());
          } else if (current_node_ptr->HasLeftChild()) {
            current_node_ptr = current_node_ptr->GetLeftChild();
            node_stack.push(current_node_ptr);
            // std::cout << "go to left child: ";
            // PrintElement(current_node_ptr->GetData());
          }
        }
      }
    }

    nearest_point = nearest_node_ptr->GetData();
    return true;
  }

  bool FindNearestPoint(const T& target_point, std::vector<T>& nearest_point_vector, const size_t k = 1) {
    if (!HasData() || !TreeValid()) {
      return false;
    }

    // recursive lambda function with no return
    auto func_dfs = [this](std::deque<KDNodePtr>& node_pq, KDNodePtr current_node_ptr, KDNodePtr target_node_ptr,
                           auto&& func_dfs) -> void {
      // struct {
      //   bool operator()(const KDNodePtr& a, const KDNodePtr& b) { return a->GetDist() <= b->GetDist(); }
      // } cmp;

      auto func_is_leaf = [](KDNodePtr node_ptr) { return !(node_ptr->HasLeftChild() || node_ptr->HasRightChild()); };

      auto func_cal_dist_sqr = [this](KDNodePtr node_ptr0, KDNodePtr node_ptr1) {
        double sqr_sum = 0.0;
        for (int i = 0; i < this->dimension_; ++i) {
          sqr_sum += std::pow(node_ptr0->GetData()[i] - node_ptr1->GetData()[i], 2);
        }
        return sqr_sum;
      };

      auto func_judge_hypersurface_cut = [](KDNodePtr current_node_ptr, KDNodePtr target_node_ptr,
                                            const double radius_sqr) {
        auto current_split = current_node_ptr->GetSplit();
        double dist = std::fabs(current_node_ptr->GetData()[current_split] - target_node_ptr->GetData()[current_split]);
        std::cout << "cur split: " << current_split << " cut dist: " << dist << " radius_sqr: " << radius_sqr
                  << std::endl;
        return dist * dist <= radius_sqr;
      };

      if (current_node_ptr == nullptr) {
        std::cout << "null & return" << std::endl;
        return;
      }
      std::cout << "current node: ";
      PrintElement(current_node_ptr->GetData());

      // calculate sqr dist and push into pq
      double min_dist_sqr = func_cal_dist_sqr(current_node_ptr, target_node_ptr);
      std::cout << "min_dist_sqr: " << min_dist_sqr << std::endl;
      current_node_ptr->SetDist(min_dist_sqr);
      node_pq.emplace_back(current_node_ptr);

      if (node_pq.size() > this->k_) {
        auto cmp = [](const KDNodePtr& a, const KDNodePtr& b) { return a->GetDist() <= b->GetDist(); };
        std::sort(node_pq.begin(), node_pq.end(), cmp);
        node_pq.pop_back();
      }

      // recursively search the half of the tree
      const auto current_node_data = current_node_ptr->GetData();
      const auto current_split = current_node_ptr->GetSplit();
      const auto target_node_data = target_node_ptr->GetData();
      bool search_direction = 0;
      if (current_node_data[current_split] > target_node_data[current_split]) {
        std::cout << "go to left child: \n";
        func_dfs(node_pq, current_node_ptr->GetLeftChild(), target_node_ptr, func_dfs);
      } else {
        std::cout << "go to right child: \n";
        func_dfs(node_pq, current_node_ptr->GetRightChild(), target_node_ptr, func_dfs);
        search_direction = 1;
      }
      std::cout << "top node: ";
      PrintElement(node_pq.front()->GetData());
      std::cout << "cur node: ";
      PrintElement(current_node_ptr->GetData());
      // If the candidate hypersphere crosses this splitting plane, look on the other side of the plane by examining the
      // other subtree.
      bool size_not_enough = node_pq.size() < this->k_;
      bool cut_with_hypersurface =
          func_judge_hypersurface_cut(current_node_ptr, target_node_ptr, node_pq.front()->GetDist());
      std::cout << "size: " << node_pq.size() << " cut: " << cut_with_hypersurface << std::endl;
      if (size_not_enough || cut_with_hypersurface) {
        if (search_direction == 0) {
          std::cout << "cut & go to right child: ";
          PrintElement(current_node_ptr->GetData());
          func_dfs(node_pq, current_node_ptr->GetRightChild(), target_node_ptr, func_dfs);
        } else {
          std::cout << "cut & go to left child: ";
          PrintElement(current_node_ptr->GetData());
          func_dfs(node_pq, current_node_ptr->GetLeftChild(), target_node_ptr, func_dfs);
        }
      }
      std::cout << "final cur node: ";
      PrintElement(current_node_ptr->GetData());
    };

    k_ = k;
    std::deque<KDNodePtr> node_pq;
    KDNodePtr target_node_ptr(new KDNode<T>);
    target_node_ptr->SetData(target_point);
    // recursive search
    func_dfs(node_pq, root_, target_node_ptr, func_dfs);

    // output to a vector
    nearest_point_vector.clear();
    nearest_point_vector.reserve(k);
    for (const auto& node_ptr : node_pq) {
      nearest_point_vector.emplace_back(node_ptr->GetData());
    }
    return true;
  }

  void PrintTree(const int method = 0) {
    if (!HasData() || !TreeValid()) {
      return;
    }
    if (method == 0) {  // bfs
      std::cout << "[PrintTree] bfs" << std::endl;
      std::queue<KDNodePtr> node_queue;
      node_queue.push(root_);
      while (!node_queue.empty()) {
        KDNodePtr node_ptr = node_queue.front();
        node_queue.pop();
        PrintElement(node_ptr->GetData());
        if (node_ptr->HasLeftChild()) {
          node_queue.push(node_ptr->GetLeftChild());
        }
        if (node_ptr->HasRightChild()) {
          node_queue.push(node_ptr->GetRightChild());
        }
      }
    } else {  // dfs
      // recursive lambda function with no return
      auto func_dfs = [this](std::stack<KDNodePtr>& node_stack, KDNodePtr cur_node_ptr, auto&& func_dfs) -> void {
        int counter = 0;
        if (cur_node_ptr->HasLeftChild()) {
          node_stack.push(cur_node_ptr->GetLeftChild());
          func_dfs(node_stack, cur_node_ptr->GetLeftChild(), func_dfs);
        }
        ++counter;
        if (cur_node_ptr->HasRightChild()) {
          node_stack.push(cur_node_ptr->GetRightChild());
          func_dfs(node_stack, cur_node_ptr->GetRightChild(), func_dfs);
        }
        ++counter;
        if (!(cur_node_ptr->HasLeftChild() || cur_node_ptr->HasRightChild()) || counter == 2) {
          this->PrintElement(cur_node_ptr->GetData());
          node_stack.pop();
        }
      };

      std::cout << "[PrintTree] dfs" << std::endl;
      std::stack<KDNodePtr> node_stack;
      node_stack.push(root_);
      func_dfs(node_stack, root_, func_dfs);
    }
    std::cout << std::endl;
  }

  // from small to big
  void InsertSort(int cur_dim, const size_t lower_idx, const size_t upper_idx) {
    if (!HasData() || lower_idx >= upper_idx || upper_idx >= DataSize()) {
      return;
    }
    for (size_t i = lower_idx + 1; i <= upper_idx; ++i) {
      auto key = elements_[i];
      size_t j = i - 1;
      bool shifted = false;
      bool zero_reached = false;
      while (key[cur_dim] < elements_[j][cur_dim]) {
        elements_[j + 1] = elements_[j];
        shifted = true;
        if (j == 0) {
          zero_reached = true;
          break;
        } else {
          --j;
        }
      }
      if (shifted) {
        if (zero_reached) {
          elements_[j] = key;
        } else {
          elements_[j + 1] = key;
        }
      }
    }
  }

  double CalculateVariation(int cur_dim, const size_t lower_idx, const size_t upper_idx) {
    if (!HasData() || lower_idx > upper_idx || upper_idx >= DataSize()) {
      return 0.0;
    }
    double mean = 0.0, var = 0.0;
    for (size_t i = lower_idx; i <= upper_idx; ++i) {
      mean += elements_[i][cur_dim];
    }
    mean /= static_cast<double>(DataSize());
    for (size_t i = lower_idx; i <= upper_idx; ++i) {
      double temp = elements_[i][cur_dim] - mean;
      var += temp * temp;
    }
    var /= static_cast<double>(DataSize());
    return var;
  }

  void PrintDataSet() {
    std::cout << "[PrintDataSet] " << std::endl;
    for (const auto& elem : elements_) {
      PrintElement(elem);
    }
    std::cout << std::endl;
  }

 private:
  std::vector<T> elements_;
  KDNodePtr root_ = nullptr;
  int dimension_ = 0;
  size_t k_ = 1;  // num of nearest points

  void PrintElement(const T& elem) {
    if (dimension_ <= 0) {
      return;
    }
    std::cout << "[";
    for (size_t i = 0; i < dimension_; ++i) {
      std::cout << elem[i];
      if (i < dimension_ - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }
};

}  // namespace lidar_localization

#endif

int main(int argc, char const *argv[])
{
  /* code */
  size_t a = 3ul;
  size_t b = 4ul;
  size_t c = (a+b)/2ul;
  std::cout << c << std::endl;

  // data_set = [[2, 3], [5, 4], [9, 6], [4, 7], [8, 1], [7, 2]]
  double data_set[6][2] = {2,3,5,4,9,6,4,7,8,1,7,2};
  lidar_localization::KDTree<Eigen::VectorXd> kd_tree;
  for (int i = 0; i < 6; ++i) {
    Eigen::VectorXd a(2);
    a(0) = data_set[i][0];
    a(1) = data_set[i][1];
    kd_tree.AddData(a,2);
  }
  kd_tree.PrintDataSet();
  kd_tree.GenerateTree();
  kd_tree.PrintTree(0);

  Eigen::VectorXd x(2);
  x(0) = 2.0;
  x(1) = 4.5;
  Eigen::VectorXd y(2);
  std::vector<Eigen::VectorXd> y_set;
  bool result = kd_tree.FindNearestPoint(x,y);
  result = kd_tree.FindNearestPoint(x,y_set,2);
  std::cout << "result: " << result << " nearest point: " << std::endl << y << std::endl;
  std::cout << "result: " << result << " nearest point: " << std::endl;
  for (auto y : y_set) {
    std::cout << y << std::endl;
  }

  return 0;
}
