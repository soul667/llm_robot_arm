#!/usr/bin/env python3
import uuid
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import Parameter

from llm_proxy.llm_client import LLMClient
from llm_proxy.analysis import LLMResponseAnalyzer
from my_interfaces.action import LLMQuery
from my_interfaces.srv import ListModels

class LLMProxyNode(Node):
    """ROS2节点，代理LLM服务请求并分析响应。"""

    def __init__(self):
        super().__init__('llm_proxy_node')
        
        # 声明基础参数
        self.declare_parameter('default_model', 'gpt35')
        self.declare_parameter('request_timeout', 30.0)
        
        # 声明模型配置参数
        model_params = [
            ('models.gpt35.name', 'gpt-3.5-turbo'),
            ('models.gpt35.api_endpoint', 'https://api.openai.com/v1/chat/completions'),
            ('models.gpt35.api_key', ''),
            ('models.gpt35.default_temperature', 0.7),
            ('models.gpt35.default_max_tokens', 1024),
            ('models.gpt35.model_format', 'openai'),
            
            ('models.gpt4.name', 'gpt-4'),
            ('models.gpt4.api_endpoint', 'https://api.openai.com/v1/chat/completions'),
            ('models.gpt4.api_key', ''),
            ('models.gpt4.default_temperature', 0.7),
            ('models.gpt4.default_max_tokens', 2048),
            ('models.gpt4.model_format', 'openai'),
            
            ('models.claude.name', 'claude-3-opus-20240229'),
            ('models.claude.api_endpoint', 'https://api.anthropic.com/v1/messages'),
            ('models.claude.api_key', ''),
            ('models.claude.default_temperature', 0.7),
            ('models.claude.default_max_tokens', 4096),
            ('models.claude.model_format', 'anthropic'),
            
            ('models.zhipu.name', 'glm-4'),
            ('models.zhipu.api_endpoint', 'https://open.bigmodel.cn/api/paas/v4/chat/completions'),
            ('models.zhipu.api_key', ''),
            ('models.zhipu.default_temperature', 0.7),
            ('models.zhipu.default_max_tokens', 2048),
            ('models.zhipu.model_format', 'openai'),
        ]
        
        for param_name, default_value in model_params:
            self.declare_parameter(param_name, default_value)
        
        # 获取基础参数
        default_model_id = self.get_parameter('default_model').value
        
        # 构建模型配置字典
        self.models_config = {}
        model_ids = ['gpt35', 'gpt4', 'claude', 'zhipu']
        
        for model_id in model_ids:
            base_path = f'models.{model_id}'
            self.models_config[model_id] = {
                'name': self.get_parameter(f'{base_path}.name').value,
                'api_endpoint': self.get_parameter(f'{base_path}.api_endpoint').value,
                'api_key': self.get_parameter(f'{base_path}.api_key').value,
                'default_temperature': float(self.get_parameter(f'{base_path}.default_temperature').value),
                'default_max_tokens': int(self.get_parameter(f'{base_path}.default_max_tokens').value),
                'model_format': self.get_parameter(f'{base_path}.model_format').value,
            }
        
        # 检查模型配置
        if not self.models_config:
            self.get_logger().error('未找到模型配置！使用空配置初始化')
            self.models_config = {}
        
        # 初始化组件
        self.llm_client = LLMClient(
            models_config=self.models_config,
            default_model_id=default_model_id
        )
        
        self.analyzer = LLMResponseAnalyzer()
        
        # 创建Action服务器
        self._action_server = ActionServer(
            self,
            LLMQuery,
            'llm_query',
            self.execute_callback
        )
        
        # 创建服务以列出可用模型
        self._list_models_service = self.create_service(
            ListModels,
            'list_available_models',
            self.list_models_callback
        )
        
        self.get_logger().info('LLM代理节点已启动，可用模型: ' + 
                              str(list(self.llm_client.list_available_models().keys())))

    def list_models_callback(self, request, response):
        """处理列出可用模型的服务请求"""
        available_models = self.llm_client.list_available_models()
        response.model_ids = list(available_models.keys())
        response.model_names = list(available_models.values())
        return response

    def execute_callback(self, goal_handle):
        """处理传入的LLM查询Action请求"""
        request = goal_handle.request
        self.get_logger().info(f"收到查询: {request.query[:50]}...")
        
        # 生成查询ID
        query_id = str(uuid.uuid4())
        
        # 准备反馈和结果
        feedback_msg = LLMQuery.Feedback()
        result = LLMQuery.Result()
        
        start_time = time.time()
        success = True
        error_msg = ""
        llm_response = ""
        analysis_result = ""
        model_id = request.model_id if request.model_id else None
        
        try:
            # 设置初始反馈
            feedback_msg.status = "正在准备请求"
            feedback_msg.progress = 0.1
            feedback_msg.stage = "preparing"
            goal_handle.publish_feedback(feedback_msg)
            
            # 获取使用的模型名称进行日志记录
            model_config = self.llm_client.get_model_config(model_id)
            model_name = model_config["name"]
            self.get_logger().info(f"使用模型: {model_name}")
            
            # 发送API请求
            feedback_msg.status = "正在调用LLM API"
            feedback_msg.progress = 0.3
            feedback_msg.stage = "calling_api"
            goal_handle.publish_feedback(feedback_msg)
            
            # 调用LLM API
            llm_response = self.llm_client.query(
                request.query,
                model_id=model_id,
                temperature=request.temperature,
                max_tokens=request.max_tokens
            )
            
            feedback_msg.status = "收到LLM响应"
            feedback_msg.progress = 0.7
            feedback_msg.stage = "response_received"
            goal_handle.publish_feedback(feedback_msg)
            
            # 如果请求，分析响应
            if request.analyze:
                feedback_msg.status = "正在分析响应"
                feedback_msg.progress = 0.8
                feedback_msg.stage = "analyzing"
                goal_handle.publish_feedback(feedback_msg)
                
                analysis_result = self.analyzer.analyze_response(
                    request.query, 
                    llm_response
                )
                
        except Exception as e:
            success = False
            error_msg = str(e)
            self.get_logger().error(f"处理LLM请求时出错: {e}")
            
        # 计算响应时间
        response_time = time.time() - start_time
            
        # 准备结果
        result.response = llm_response
        result.analysis = analysis_result
        result.success = success
        result.error_msg = error_msg
        result.response_time = response_time
        
        # 发布最终反馈
        feedback_msg.status = "完成"
        feedback_msg.progress = 1.0
        feedback_msg.stage = "completed"
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info(f"查询 {query_id} 在 {response_time:.2f}秒内处理完成")
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LLMProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
