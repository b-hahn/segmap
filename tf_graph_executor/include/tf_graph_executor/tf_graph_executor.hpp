// This code was originally written by Martin Pecka ( martin.pecka@cvut.cz ) and adapted
// for our application.

#ifndef TF_GRAPH_EXECUTOR_TF_GRAPH_EXECUTOR_HPP
#define TF_GRAPH_EXECUTOR_TF_GRAPH_EXECUTOR_HPP

#include <string>
#include <vector>
#include <memory>

// We need to use tensorflow::* classes as PIMPL
namespace tensorflow {
class Session;
class Status;
class Tensor;
class MetaGraphDef;
};

namespace tf_graph_executor {

struct Array3D {
  Array3D(unsigned int x_dim, unsigned int y_dim, unsigned int z_dim) {
    resize(x_dim, y_dim, z_dim);
    init();
  }

  void resize(unsigned int x_dim, unsigned int y_dim, unsigned int z_dim) {
    container.resize(x_dim);
    for (auto& y : container) {
      y.resize(y_dim);
      for (auto& z : y) {
        z.resize(z_dim);
      }
    }
  }

  void init() {
    for (auto& x : container) {
      for (auto& y : x) {
        for (auto& z : y) {
          z = 0.0;
        }
      }
    }
  }

  std::vector<std::vector<std::vector<double> > > container;
};

// TODO: make this templated
struct Array4D
{
    Array4D(unsigned int x_dim, unsigned int y_dim, unsigned int z_dim, unsigned int color_dim)
    {
        resize(x_dim, y_dim, z_dim, color_dim);
        init();
    }

    void resize(unsigned int x_dim, unsigned int y_dim, unsigned int z_dim, unsigned int color_dim)
    {
        container.resize(x_dim);
        for (auto& y : container) {
            y.resize(y_dim);
            for (auto& z : y) {
                z.resize(z_dim);
                // don't need to resize color_dim since it always stays the same (==4)
                for (auto& c : z) {
                    c.resize(color_dim);
                }
            }
        }
    }

    void init()
    {
        for (auto& x : container) {
            for (auto& y : x) {
                for (auto& z : y) {
                    // TODO: couldn't I use uint8_t's here?
                    z = std::vector<double>{ 0.0, 0.0, 0.0, 0.0 };
                }
            }
        }
    }

    std::vector<std::vector<std::vector<std::vector<double>>>> container;
};

class TensorflowGraphExecutor {
public:
    explicit TensorflowGraphExecutor(const std::string& pathToGraph);

    virtual ~TensorflowGraphExecutor();

    void loadCheckpoint(const std::string& checkpointPath);

    std::vector<float> executeGraph(const std::vector<float>& inputs,
                                    const std::string& input_tensor_name,
                                    const std::string& output_tensor_name) const;

    std::vector<std::vector<float> > batchExecuteGraph(
        const std::vector<std::vector<float> >& inputs, const std::string& input_tensor_name,
        const std::string& output_tensor_name) const;

    template <typename NN_INPUT>
    std::vector<std::vector<float> > batchExecuteGraph(
        const std::vector<NN_INPUT>& inputs, const std::string& input_tensor_name,
        const std::string& output_tensor_name) const;


  template <typename NN_INPUT>
  void batchFullForwardPass(
      const std::vector<NN_INPUT>& inputs,
      const std::string& input_tensor_name,
      const std::vector<std::vector<float> >& scales,
      const std::string& scales_tensor_name,
      const std::string& descriptor_values_name,
      const std::string& reconstruction_values_name,
      std::vector<std::vector<float> >& descriptors,
      std::vector<Array3D>& reconstructions,
      const std::shared_ptr<std::vector<float>> semantic_segmentation = nullptr,
      const std::string& semantic_segmentation_tensor_name = "") const;

//   // template specialization for 3D neural net input 
//   template <>
//   void batchFullForwardPass(
//       const std::vector<Array3D>& inputs,
//       const std::string& input_tensor_name,
//       const std::vector<std::vector<float> >& scales,
//       const std::string& scales_tensor_name,
//       const std::string& descriptor_values_name,
//       const std::string& reconstruction_values_name,
//       std::vector<std::vector<float> >& descriptors,
//       std::vector<Array3D>& reconstructions,
//       const std::shared_ptr<std::vector<std::vector<float> >> semantic_segmentation = nullptr,
//       const std::string& semantic_segmentation_tensor_name = "") const;

//   // template specialization for 4D neural net input 
//   template <>
//   void batchFullForwardPass(
//       const std::vector<Array4D>& inputs,
//       const std::string& input_tensor_name,
//       const std::vector<std::vector<float> >& scales,
//       const std::string& scales_tensor_name,
//       const std::string& descriptor_values_name,
//       const std::string& reconstruction_values_name,
//       std::vector<std::vector<float> >& descriptors,
//       std::vector<Array3D>& reconstructions,
//       const std::shared_ptr<std::vector<std::vector<float> >> semantic_segmentation = nullptr,
//       const std::string& semantic_segmentation_tensor_name = "") const;

    tensorflow::Status executeGraph(const tensorflow::Tensor& inputTensor,
                                    tensorflow::Tensor& outputTensor,
                                    const std::string& input_tensor_name,
                                    const std::string& output_tensor_name) const;

    tensorflow::Status executeGraph(const std::vector<std::pair<std::string, tensorflow::Tensor> >& feedDict,
                                    const std::vector<std::string>& outputOps,
                                    std::vector<tensorflow::Tensor>& outputTensors) const;

protected:
    tensorflow::Session* tensorflowSession;

    tensorflow::MetaGraphDef* graph_def;
};

}


#endif //TF_GRAPH_EXECUTOR_TF_GRAPH_EXECUTOR_HPP
