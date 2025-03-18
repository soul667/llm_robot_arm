#!/usr/bin/env python3
import json
import os
import requests
from typing import Dict, Any, Optional
from dotenv import load_dotenv

# 加载环境变量
load_dotenv()

class LLMClient:
    """支持多模型的LLM API客户端。"""
    
    def __init__(self, models_config: Dict[str, Any], default_model_id: str = "gpt35"):
        """
        初始化LLM客户端，加载多个模型配置。
        
        Args:
            models_config: 模型配置字典，包含每个预设模型的详细配置
            default_model_id: 默认要使用的模型ID
        """
        self.models_config = models_config
        self.default_model_id = default_model_id
        
        # 验证默认模型ID存在
        if default_model_id not in self.models_config:
            available_models = list(self.models_config.keys())
            raise ValueError(f"默认模型ID '{default_model_id}'不存在。可用模型: {available_models}")
    
    def get_model_config(self, model_id: Optional[str] = None) -> Dict[str, Any]:
        """获取指定模型的配置，如果未指定则使用默认模型。优先使用环境变量中的API密钥。"""
        model_id = model_id if model_id and model_id in self.models_config else self.default_model_id
        config = self.models_config[model_id].copy()
        
        # 根据模型格式获取对应的环境变量API密钥
        if config["model_format"] == "openai":
            env_key = os.getenv("OPENAI_API_KEY")
        elif config["model_format"] == "anthropic":
            env_key = os.getenv("ANTHROPIC_API_KEY")
        elif config["model_format"] == "openai" and "zhipu" in model_id:
            env_key = os.getenv("ZHIPU_API_KEY")
            
        # 如果环境变量中有API密钥，则使用环境变量中的值
        if env_key:
            config["api_key"] = env_key
        elif not config["api_key"]:
            raise ValueError(f"未找到模型 {model_id} 的API密钥。请在环境变量或配置文件中设置。")
            
        return config
    
    def list_available_models(self) -> Dict[str, str]:
        """返回所有可用模型的ID和名称。"""
        return {model_id: config["name"] for model_id, config in self.models_config.items()}
    
    def query(
        self, 
        prompt: str,
        model_id: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None
    ) -> str:
        """
        向LLM API发送查询并返回响应。
        
        Args:
            prompt: 输入文本提示
            model_id: 要使用的模型ID（如不指定则使用默认模型）
            temperature: 响应生成的温度参数（如不指定则使用模型默认值）
            max_tokens: 响应中的最大token数（如不指定则使用模型默认值）
            
        Returns:
            LLM响应文本
        
        Raises:
            Exception: API请求失败时抛出
        """
        # 获取模型配置
        model_config = self.get_model_config(model_id)
        model_name = model_config["name"]
        api_endpoint = model_config["api_endpoint"]
        api_key = model_config["api_key"]
        model_format = model_config["model_format"]
        
        # 使用模型默认值或指定值
        temp = temperature if temperature is not None else model_config["default_temperature"]
        max_tok = max_tokens if max_tokens is not None else model_config["default_max_tokens"]
        timeout = model_config.get("timeout", 30.0)
        
        # API请求头
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}"
        }
        
        # 根据不同的API格式构建请求负载
        if model_format == "openai":
            payload = {
                "model": model_name,
                "messages": [{"role": "user", "content": prompt}],
                "temperature": temp,
                "max_tokens": max_tok
            }
            
            # 发送API请求
            try:
                response = requests.post(
                    api_endpoint,
                    headers=headers,
                    data=json.dumps(payload),
                    timeout=timeout
                )
                
                # 检查响应是否成功
                response.raise_for_status()
                
                # 解析响应JSON
                response_data = response.json()
                
                # 提取并返回响应文本
                return response_data["choices"][0]["message"]["content"]
                
            except requests.exceptions.RequestException as e:
                raise Exception(f"API请求失败: {str(e)}")
            except (KeyError, IndexError, json.JSONDecodeError) as e:
                raise Exception(f"解析API响应失败: {str(e)}")
                
        elif model_format == "anthropic":
            # Claude API格式
            payload = {
                "model": model_name,
                "messages": [{"role": "user", "content": prompt}],
                "temperature": temp,
                "max_tokens": max_tok
            }
            
            # Claude API可能需要特殊头部
            headers["anthropic-version"] = "2023-06-01"
            
            try:
                response = requests.post(
                    api_endpoint,
                    headers=headers,
                    data=json.dumps(payload),
                    timeout=timeout
                )
                
                response.raise_for_status()
                response_data = response.json()
                
                # Claude API的响应格式
                return response_data["content"][0]["text"]
                
            except requests.exceptions.RequestException as e:
                raise Exception(f"API请求失败: {str(e)}")
            except (KeyError, IndexError, json.JSONDecodeError) as e:
                raise Exception(f"解析API响应失败: {str(e)}")
        
        else:
            raise ValueError(f"不支持的模型格式: {model_format}")
