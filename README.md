# 二轮差速运动模型的ROS驱动

---

## 引言

本项目为完成一个轮椅的ROS驱动，以在该轮椅上实现导航的功能。
这也是我第一次在项目开发中使用git，我希望通过该项目，提高我对于git，以及Python、Markdown等工具的使用能力，同时有意识的提高代码的规范性，当然我也会再次强调，我觉得代码最核心的内容是代码本身，代码思想本身的简洁是最高的追求，此外，项目开发的效率也是放在首位的，技术只是服务。

## 计划

- 时间上来说，我希望在本周完成第一版本的驱动，起码能够成功的通过键盘控制轮椅移动，转速看着正确；争取在第二周跑通建图和导航算法。
- **Step by Step, But Efficiency**

## 理论

二轮差速运动模型的运动学公式如下：

1. **轮子线速度**  
   左右轮的线速度分别由轮子半径 \( r \) 和角速度 \( \omega_{\text{left}} \)、\( \omega_{\text{right}} \) 决定：  
   \[
   v_{\text{left}} = r \cdot \omega_{\text{left}}, \quad v_{\text{right}} = r \cdot \omega_{\text{right}}
   \]

2. **机器人线速度**  
   机器人的整体线速度 \( v \) 为左右轮线速度的平均值：  
   \[
   v = \frac{v_{\text{left}} + v_{\text{right}}}{2} = \frac{r}{2} \left( \omega_{\text{left}} + \omega_{\text{right}} \right)
   \]

3. **机器人角速度**  
   角速度 \( \omega \) 由左右轮线速度差和轮距 \( L \) 决定：  
   \[
   \omega = \frac{v_{\text{right}} - v_{\text{left}}}{L} = \frac{r}{L} \left( \omega_{\text{right}} - \omega_{\text{left}} \right)
   \]

4. **位姿微分方程**  
   机器人位置 \((x, y)\) 和朝向 \(\theta\) 随时间的变化率：  
   \[
   \frac{dx}{dt} = v \cos\theta, \quad \frac{dy}{dt} = v \sin\theta, \quad \frac{d\theta}{dt} = \omega
   \]

5. **逆运动学公式**  
   根据期望的线速度 \( v \) 和角速度 \( \omega \)，计算左右轮角速度：  
   \[
   \omega_{\text{left}} = \frac{2v - L\omega}{2r}, \quad \omega_{\text{right}} = \frac{2v + L\omega}{2r}
   \]

6. **转弯半径**  
   瞬时转弯半径 \( R \)（当 \( \omega \neq 0 \) 时）：  
   \[
   R = \frac{v}{\omega}
   \]

**参数说明**：  

- \( L \): 轮距（左右轮中心距离）  
- \( r \): 轮子半径  
- \( \omega_{\text{left}}, \omega_{\text{right}} \): 左右轮角速度  
- \( v_{\text{left}}, v_{\text{right}} \): 左右轮线速度  
- \( v \): 机器人线速度  
- \( \omega \): 机器人角速度  
- \( \theta \): 机器人朝向角（相对于全局坐标系）  

这些公式完整描述了二轮差速驱动机器人的运动学关系，适用于运动控制、路径规划及状态估计等领域。
