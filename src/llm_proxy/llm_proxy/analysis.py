#!/usr/bin/env python3
from typing import Optional

class LLMResponseAnalyzer:
    """分析LLM响应的实用工具类。"""
    
    def analyze_response(self, query: str, response: str) -> str:
        """
        分析LLM的查询和响应。

        Args:
            query: 原始查询文本
            response: LLM的响应文本

        Returns:
            分析结果的文本描述
        """
        analysis_points = []
        
        # 基本信息统计
        response_words = len(response.split())
        query_words = len(query.split())
        analysis_points.append(f"响应长度: {response_words}词")
        analysis_points.append(f"查询长度: {query_words}词")
        
        # 响应完整性检查
        if response.strip():
            if response[-1] in ".!?。！？":
                analysis_points.append("响应完整性: 完整")
            else:
                analysis_points.append("响应完整性: 可能不完整")
        else:
            analysis_points.append("响应完整性: 空响应")
            
        # 合并分析结果
        return "\n".join(analysis_points)
