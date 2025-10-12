# Computer Vision Skill Evaluation Template

Quick reference for evaluating computer vision knowledge level.

## Evaluation Questions

### Level 1: Beginner Check

**Q1**: What is a pixel and how are images stored in memory?
- ✅ Pass: Picture element, array of numbers, explains RGB/grayscale
- ❌ Fail: Vague or incorrect

**Q2**: What's the difference between RGB and grayscale?
- ✅ Pass: 3 channels vs 1, color vs intensity only
- ❌ Fail: Can't explain clearly

### Level 2: Novice Check

**Q3**: Why might you convert from RGB to HSV for object detection?
- ✅ Pass: HSV separates color from brightness, easier to threshold colors
- ⚠️ Partial: Knows about color spaces but not why
- ❌ Fail: Doesn't understand color spaces

**Q4**: Explain what edge detection is and what it's used for.
- ✅ Pass: Finds boundaries/discontinuities, used for shape detection/segmentation
- ⚠️ Partial: Vague understanding
- ❌ Fail: Doesn't know

**Q5**: You need to process a 30 FPS video stream. What affects whether your algorithm is fast enough?
- ✅ Pass: Image size, algorithm complexity, hardware, mentions optimization
- ⚠️ Partial: Mentions some factors but incomplete
- ❌ Fail: No understanding of performance

### Level 3: Advanced Beginner Check

**Q6**: Explain feature detection and when you'd use it vs template matching.
- ✅ Pass: Features = keypoints, scale/rotation invariant, template = fixed
- ⚠️ Partial: Heard of both but fuzzy on differences
- ❌ Fail: Doesn't know feature detection

**Q7**: How does camera calibration work and why is it needed?
- ✅ Pass: Corrects distortion, intrinsic/extrinsic parameters, explains need
- ⚠️ Partial: Knows it's needed but not how
- ❌ Fail: Doesn't know about calibration

**Q8**: What is optical flow and what is it used for?
- ✅ Pass: Motion between frames, tracking, explains use cases
- ⚠️ Partial: Heard of it but vague
- ❌ Fail: Doesn't know

### Level 4: Intermediate Check

**Q9**: Compare SIFT, ORB, and AKAZE features. When would you use each?
- ✅ Pass: Explains trade-offs (accuracy vs speed), knows use cases
- ⚠️ Partial: Knows names but not differences
- ❌ Fail: Doesn't know multiple methods

**Q10**: How do you optimize a CNN for real-time inference on embedded hardware?
- ✅ Pass: Quantization, pruning, model selection, hardware acceleration
- ⚠️ Partial: Some ideas but not comprehensive
- ❌ Fail: No optimization knowledge

## Code Reading Test

Show this code:

```python
import cv2
import numpy as np

img = cv2.imread('image.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

mask = cv2.inRange(hsv, lower_red, upper_red)
result = cv2.bitwise_and(img, img, mask=mask)
```

**Questions**:
1. What does this code do?
2. Why convert to HSV?
3. What do the numbers in lower_red and upper_red represent?
4. What would `result` contain?

**Scoring**:
- ✅ Advanced Beginner+: Answers all correctly
- ⚠️ Novice: Gets 2-3 correct
- ❌ Beginner: Gets 0-1 correct

## Practical Challenge

**Beginner**: "How would you find all red pixels in an image?"
- ✅ Novice+: Mentions color thresholding or similar

**Novice**: "Describe how to detect a red ball in an image."
- ✅ Advanced Beginner+: Color threshold → morphology → contours → circle detection

**Advanced Beginner**: "How would you track a moving object across frames?"
- ✅ Intermediate+: Mentions optical flow, Kalman filter, or detection + matching

## Level Determination

**Count passes**:
- Q1-Q2: Both pass → Above Beginner
- Q3-Q5: 2+ pass → Above Novice
- Q6-Q8: 2+ pass → Above Advanced Beginner
- Q9-Q10: Both pass → Intermediate+
- Code test: 3+ correct → +1 level
- Practical challenge: Completed → Confirms level

## Recommended Starting Points

**Complete Beginner**:
- Day 1: Image basics, loading, displaying
- Day 2: Pixel manipulation, color spaces
- Very slow pace, foundational concepts

**Novice**:
- Day 1: Quick review, color space conversion
- Day 2: Thresholding and basic filtering
- Standard pace

**Advanced Beginner**:
- Day 1: Quick review, jump to feature detection
- Day 3-4: Start with object tracking
- Faster pace

**Intermediate**:
- Day 5-8: Start with deep learning/advanced topics
- Skip OpenCV basics
- Fast pace, complex projects

## Gap Identification

- Failed Q3 → Color space deep dive needed
- Failed Q4 → Edge detection fundamentals
- Failed Q5 → Performance optimization practice
- Failed Q6 → Feature detection tutorial needed
- Failed Q7 → Camera calibration exercise
- Failed Q8 → Optical flow introduction
- Failed Q9 → Feature comparison study
- Failed Q10 → ML optimization workshop needed
