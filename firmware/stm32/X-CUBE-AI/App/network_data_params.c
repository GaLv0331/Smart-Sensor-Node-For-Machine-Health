/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-01-25T23:08:26+0530
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_weights_array_u64[59] = {
  0xbea4c5ea3e68c98fU, 0xbd335816bee310e8U, 0x3f41922d3ceda045U, 0xbe88998f3ed079afU,
  0x3e0a5cc8bea5b0f4U, 0x3f17021d3e768ce8U, 0x3e03a8e6bf2da702U, 0xbf6fac00beec2760U,
  0xbe1b5a8cbe82a1b0U, 0x3dd8a40d3e175e35U, 0x3ccef70c3ed0c143U, 0x3e2974163e5b33c8U,
  0x3f0166693eca0dc2U, 0x3de0cb18bf44947dU, 0x3f535c57bdaddac1U, 0x3eb0da6fbed37126U,
  0x3e0a7cbb3f1f0a3cU, 0x3f0c9ec63f469f21U, 0x3f31f1c6bed61e8aU, 0xbee428803ecad286U,
  0xbe803256be83b00eU, 0x3f5633ab3ed1aab0U, 0xbf361e4f3e7b5f40U, 0x3f389e313ccefd2fU,
  0xbdb59d41bed4ce45U, 0x3e48af3e3eac918eU, 0x3e1b108a3f509c99U, 0x3f319d443f3af685U,
  0x3decc18b3e9cf352U, 0xbd1290dd3e4a4cf2U, 0xbd9dde813f13ef13U, 0x3eaa70c63f22bb9fU,
  0x3ee883313f6c415dU, 0xbf31e5d73f4738d3U, 0x3f200bf3bde13508U, 0x3e90da343e20745fU,
  0xbefeedf43e0ce1c4U, 0xbe1ad2e0bf2ae619U, 0x3f55373d3ed99c9fU, 0xbe69435abe31fcd6U,
  0xbe48a1653f3cad9bU, 0x3f30ce8c3f01d2e1U, 0x3c48b338be697efdU, 0xbebc5f7f3e36dd9dU,
  0x3eb843c5bf3a50aaU, 0x3f379a0fbd1b4d50U, 0x3dcfca22bd85e755U, 0xbd267ff5U,
  0xbdd899353ddc0e07U, 0x3e29aab6bd1800c3U, 0x3f00e0083db1cbf2U, 0x3eccb2e43e532295U,
  0xbe9721453f4fc5b9U, 0xbf3b8181bed303bdU, 0xbe536f0bbf2ebb6fU, 0x3f179f5e3d842398U,
  0xbe721dc23eeb2ba2U, 0x3f378e8fbe7b7ef0U, 0x3e36e0323dc19525U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

