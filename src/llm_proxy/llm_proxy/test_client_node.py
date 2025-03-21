#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import sys
import codecs
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from my_interfaces.action import LLMQuery

class TestClientNode(Node):
    """测试节点，用于发送LLM查询请求并展示结果"""

    def __init__(self):
        super().__init__('test_client_node')
        
        # 使用ReentrantCallbackGroup以支持并发
        callback_group = ReentrantCallbackGroup()
        
        self._action_client = ActionClient(
            self,
            LLMQuery,
            'llm_query',
            callback_group=callback_group
        )

    async def send_query(self, query_text, model_id="", temperature=0.7,
                        max_tokens=2048, analyze=True):
        """发送LLM查询请求
        
        Args:
            query_text (str): 要发送给LLM的文本
            model_id (str, optional): 要使用的模型ID. Defaults to "".
            temperature (float, optional): 生成参数. Defaults to 0.7.
            max_tokens (int, optional): 最大token数. Defaults to 2048.
            analyze (bool, optional): 是否分析响应. Defaults to True.
        """
        self.get_logger().info('等待Action服务器...'.encode('utf-8').decode('utf-8'))
        self._action_client.wait_for_server()
        
        # 创建目标
        goal_msg = LLMQuery.Goal()
        goal_msg.query = query_text
        goal_msg.model_id = model_id
        goal_msg.temperature = temperature
        goal_msg.max_tokens = max_tokens
        goal_msg.analyze = analyze
        
        self.get_logger().info('发送查询请求...'.encode('utf-8').decode('utf-8'))
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # 等待目标被接受
        try:
            goal_handle = await send_goal_future
            if not goal_handle.accepted:
                self.get_logger().error('目标被拒绝'.encode('utf-8').decode('utf-8'))
                return None
                
            self.get_logger().info('目标被接受'.encode('utf-8').decode('utf-8'))
            
            # 等待结果
            result_future = goal_handle.get_result_async()
            result = await result_future
            
            return result.result
            
        except Exception as e:
            self.get_logger().error(f'处理过程中出错: {str(e)}'.encode('utf-8').decode('utf-8'))
            return None

    def feedback_callback(self, feedback_msg):
        """处理反馈消息"""
        feedback = feedback_msg.feedback
        msg = f'进度: {feedback.progress:.1%} - 阶段: {feedback.stage} - 状态: {feedback.status}'
        self.get_logger().info(msg.encode('utf-8').decode('utf-8'))

async def run_test(node):
    """运行测试查询"""
    try:
        query_text = "你好，请做个自我介绍"
        result = await node.send_query(query_text)
        if result:
            node.get_logger().info('\n=== 查询结果 ==='.encode('utf-8').decode('utf-8'))
            node.get_logger().info(f'响应: \n{result.response}\n'.encode('utf-8').decode('utf-8'))
            if result.analysis:
                node.get_logger().info(f'分析: \n{result.analysis}\n'.encode('utf-8').decode('utf-8'))
            node.get_logger().info(f'处理时间: {result.response_time:.2f}秒'.encode('utf-8').decode('utf-8'))
            if not result.success:
                node.get_logger().error(f'错误: {result.error_msg}'.encode('utf-8').decode('utf-8'))
    except Exception as e:
        node.get_logger().error(f'测试运行出错: {str(e)}'.encode('utf-8').decode('utf-8'))

def main(args=None):
    # 设置标准输出编码为UTF-8
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer)
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer)
    
    rclpy.init(args=args)
    node = TestClientNode()
    
    try:
        # 创建事件循环并运行测试
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run_test(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
