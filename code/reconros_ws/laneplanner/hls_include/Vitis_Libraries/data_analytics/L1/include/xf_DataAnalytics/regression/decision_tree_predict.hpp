/*
 * Copyright 2019 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file decision_tree_predict.hpp
 * @brief Decision Tree predict function implementation.
 *
 * This file is part of Vitis ML Library.
 */
#ifndef _XF_DATA_ANALYTICS_L1_REGRESSION_DECISIONTREE_PREDICT_HPP_
#define _XF_DATA_ANALYTICS_L1_REGRESSION_DECISIONTREE_PREDICT_HPP_
#include "xf_DataAnalytics/regression/decision_tree_L1.hpp"
#include "xf_DataAnalytics/common/utils.hpp"

namespace xf {
namespace data_analytics {
namespace regression {
namespace internal {

/**
 * @brief predictFun implements the detailed predict functions.
 *
 * This function read one sample, and output a category id
 *
 * @tparam MType The data type of sample
 * @tparam WD The width of data type MType, can get by sizeof(MType)
 * @tparam MAX_FEA_NUM Max feature num function can support
 * @tparam MAX_TREE_DEPTH Max tree depth function can support
 *
 * @param onesample One sample
 * @param nodes Decision tree (Decision Tree node array)
 * @param cat_id Category id
 */
template <typename MType, unsigned WD, unsigned MAX_FEA_NUM, unsigned MAX_TREE_DEPTH>
void predictFun(ap_uint<WD> onesample[MAX_FEA_NUM],
                struct NodeR<MType> nodes[(MAX_TREE_DEPTH + 1) / 2 + 1][MAX_NODES_NUM],
                MType& reg_value) {
#pragma HLS inline
    ap_uint<MAX_TREE_DEPTH> node_id = 0;
statics_predict_loop:
    for (unsigned i = 0; i < MAX_TREE_DEPTH; i++) {
        unsigned tree_id = i >> 1;
        ap_uint<72> nodeInfo = nodes[tree_id][node_id].nodeInfo;
        if (!nodeInfo.range(0, 0)) {
            ap_uint<8> feature_id = nodeInfo.range(23, 16);
            f_cast<MType> feature_val_;
            feature_val_.i = onesample[feature_id];
            MType feature_val = feature_val_.f;
            MType threshold = nodes[tree_id][node_id].threshold;
            if (feature_val <= threshold) {
                node_id = nodeInfo.range(71, 32);
            } else {
                node_id = nodeInfo.range(71, 32) + 1;
            }
        }
    }
    reg_value = nodes[(MAX_TREE_DEPTH + 1) / 2 + 1 - 1][node_id].regValue;
}

/**
 * @brief getPredictions wrap predictFun to implement batch samples predicion.
 *
 * This function read sample streams, and output prediction result into a stream
 *
 * @tparam MType The data type of sample
 * @tparam WD The width of data type MType, can get by sizeof(MType)
 * @tparam MAX_FEA_NUM Max feature num function can support
 * @tparam MAX_TREE_DEPTH Max tree depth function can support
 *
 * @param dstrm_batch Input data streams of ap_uint<WD>
 * @param estrm_batch End flag stream for input data
 * @param nodes Decision tree (Decision Tree node array)
 * @param predictionsStrm Output data streams
 * @param predictionsTagStrm End flag stream for output
 */
template <typename MType, unsigned WD, unsigned MAX_FEA_NUM, unsigned MAX_TREE_DEPTH>
void getPredictions(hls::stream<ap_uint<WD> > dstrm_batch[MAX_FEA_NUM],
                    hls::stream<bool>& estrm_batch,
                    struct NodeR<MType> nodes[(MAX_TREE_DEPTH + 1) / 2 + 1][MAX_NODES_NUM],
                    hls::stream<MType>& predictionsStrm,
                    hls::stream<bool>& predictionsTag) {
    bool e = estrm_batch.read();
    ap_uint<WD> onesample_all_batch[MAX_FEA_NUM];
#pragma HLS array_partition variable = onesample_all_batch dim = 0 complete
    while (e) {
#pragma HLS pipeline
#pragma HLS loop_tripcount min = 1000000 max = 1000000 avg = 1000000
        for (int i = 0; i < MAX_FEA_NUM; i++) {
            onesample_all_batch[i] = dstrm_batch[i].read();
        }
        MType reg_value;
        predictFun<MType, WD, MAX_FEA_NUM, MAX_TREE_DEPTH>(onesample_all_batch, nodes, reg_value);
        predictionsStrm.write(reg_value);
        predictionsTag.write(true);
        e = estrm_batch.read();
    }
    predictionsTag.write(false);
}

/**
 * @brief getTree loads 512-bit decision tree into a list of NodeR
 *
 * Note that nodes is duplicated (max_tree_depth+1)/2+1 times for unroll each tree layer in getPredictions.
 *
 * @tparam MType The data type of sample
 * @tparam MAX_TREE_DEPTH Max tree depth function can support
 *
 * @param tree_strm Decision tree streams
 * @param eTag End flag stream for decision tree nodes
 * @param nodes Decision tree (Decision Tree node array)
 */
template <typename MType, unsigned MAX_TREE_DEPTH>
void getTree(hls::stream<ap_uint<512> >& tree_strm,
             hls::stream<bool>& eTag,
             struct NodeR<MType> nodes[(MAX_TREE_DEPTH + 1) / 2 + 1][MAX_NODES_NUM]) {
    bool e = eTag.read();
    ap_uint<512> tmp;
    int nodes_num = 0;
    while (e) {
#pragma HLS loop_tripcount min = 1024 max = 1024 avg = 1024
#pragma HLS pipeline
        tmp = tree_strm.read();
        for (int i = 0; i < (MAX_TREE_DEPTH + 1) / 2 + 1; i++) {
            for (int j = 0; j < 2; j++) {
                int offset = j * 256;
                nodes[i][nodes_num + j].nodeInfo = tmp.range(offset + 72, offset);
                f_cast<MType> thre, regV;
                regV.i = tmp.range(offset + 135, offset + 72);
                thre.i = tmp.range(offset + 255, offset + 192);
                nodes[i][nodes_num + j].regValue = regV.f;
                nodes[i][nodes_num + j].threshold = thre.f;
            }
        }
        nodes_num += 2;
        e = eTag.read();
    }
}

} // namespace internal
} // namespace regression
} // name space data_analytic
} // namespace xf

namespace xf {
namespace data_analytics {
namespace regression {
/**
 * @brief decisionTreePredict, Top function of Decision Tree Predict.
 *
 * This function first loads decision tree (the corresponding function : getTree) from treeStrm
 * Then, read sample one by one from dstrm_batch, and output its category id into predictionsStrm streams
 *
 * Note that the treeStrm is a 512-bit stream, and each 512 bits include two nodes.
 * In each 512-bit confirm the range(0,71) is node[i].nodeInfo and range(256,327) is node[i+1].nodeInfo
 *                         the range(72,135) is node[i].regValue and range(328,391) is node[i+1].regValue
 *                         the range(192,255) is node[i].threshold and range(448,511) is node[i+1].threshold
 * For detailed info of NodeR struct, can refer "decision_tree_L1.hpp"
 * Samples in input sample stream should be converted into ap_uint<WD> from MType
 *
 * @tparam MType The data type of sample
 * @tparam WD The width of data type MType, can get by sizeof(MType)
 * @tparam MAX_FEA_NUM The max feature num function can support
 * @tparam MAX_TREE_DEPTH The max tree depth function can support
 *
 * @param dstrm_batch Input data streams of ap_uint<WD>
 * @param estrm_batch End flag stream for input data
 * @param treeStrm Decision tree streams
 * @param treeTag End flag stream for decision tree nodes
 * @param predictionsStrm Output regression value streams
 * @param predictionsTagStrm End flag stream for output
 */
template <typename MType, unsigned int WD, unsigned int MAX_FEA_NUM, unsigned int MAX_TREE_DEPTH = 10>
void decisionTreePredict(hls::stream<ap_uint<WD> > dstrm_batch[MAX_FEA_NUM],
                         hls::stream<bool>& estrm_batch,
                         hls::stream<ap_uint<512> >& treeStrm,
                         hls::stream<bool>& treeTag,
                         hls::stream<MType>& predictionsStrm,
                         hls::stream<bool>& predictionsTag) {
#ifndef __SYNTHESIS__
    struct NodeR<MType>(*nodes)[MAX_NODES_NUM] = new struct NodeR<MType>[ (MAX_TREE_DEPTH + 1) / 2 + 1 ][MAX_NODES_NUM];
#else
    struct NodeR<MType> nodes[(MAX_TREE_DEPTH + 1) / 2 + 1][MAX_NODES_NUM];
#pragma HLS array_partition variable = nodes dim = 1
#pragma HLS resource variable = nodes core = RAM_2P_URAM
#endif
    internal::getTree<MType, MAX_TREE_DEPTH>(treeStrm, treeTag, nodes);
    internal::getPredictions<MType, WD, MAX_FEA_NUM, MAX_TREE_DEPTH>(dstrm_batch, estrm_batch, nodes, predictionsStrm,
                                                                     predictionsTag);
}
}
}
}
#endif
