#include "SkeletonData.h"
#include <set>
#include <cmath>  // for std::abs function
#include <algorithm> 

enum class CurveYType {
    R1, G1, B1, A1, R2, G2, B2, V1, V2, V3, V4, V5, V6, ZeroOne
}; 

const static std::vector<size_t> CurveYTypeBezierIndex = {
    0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 0
};

struct BezierCurve {
    float cx1, cy1, cx2, cy2, x1, y1, x2, y2; 
}; 

void convertBezierCurve3xTo4x(BezierCurve& bezier) {
    float timeRange = bezier.x2 - bezier.x1;
    float valueRange = bezier.y2 - bezier.y1;
    bezier.cx1 = bezier.x1 + bezier.cx1 * timeRange; 
    bezier.cy1 = bezier.y1 + bezier.cy1 * valueRange; 
    bezier.cx2 = bezier.x1 + bezier.cx2 * timeRange; 
    bezier.cy2 = bezier.y1 + bezier.cy2 * valueRange;
}

void convertBezierCurve4xTo3x(BezierCurve& bezier) {
    float timeRange = bezier.x2 - bezier.x1;
    float valueRange = bezier.y2 - bezier.y1;
    bezier.cx1 = timeRange ? (bezier.cx1 - bezier.x1) / timeRange : 0.0f;
    bezier.cy1 = valueRange ? (bezier.cy1 - bezier.y1) / valueRange : 0.0f;
    bezier.cx2 = timeRange ? (bezier.cx2 - bezier.x1) / timeRange : 1.0f;
    bezier.cy2 = valueRange ? (bezier.cy2 - bezier.y1) / valueRange : 1.0f;
}

void convertTimelineCurve3xTo4x(Timeline& timeline, std::vector<CurveYType> curveYTypes) {
    for (size_t i = 0; i + 1 < timeline.size(); i++) {
        auto& frame = timeline[i];
        if (frame.curveType == CurveType::CURVE_BEZIER) {
            float cx1 = frame.curve[0];
            float cy1 = frame.curve[1];
            float cx2 = frame.curve[2];
            float cy2 = frame.curve[3];
            for (CurveYType curveYType : curveYTypes) {
                size_t bezierIndex = CurveYTypeBezierIndex[(size_t)curveYType];
                if (frame.curve.size() < (bezierIndex + 1) * 4) {
                    frame.curve.resize((bezierIndex + 1) * 4);
                }
                BezierCurve bezier;
                bezier.cx1 = cx1;
                bezier.cy1 = cy1;
                bezier.cx2 = cx2;
                bezier.cy2 = cy2;
                bezier.x1 = frame.time;
                switch (curveYType) {
                    case CurveYType::R1: bezier.y1 = frame.color1.value().r / 255.0f; break;
                    case CurveYType::G1: bezier.y1 = frame.color1.value().g / 255.0f; break;
                    case CurveYType::B1: bezier.y1 = frame.color1.value().b / 255.0f; break;
                    case CurveYType::A1: bezier.y1 = frame.color1.value().a / 255.0f; break;
                    case CurveYType::R2: bezier.y1 = frame.color2.value().r / 255.0f; break;
                    case CurveYType::G2: bezier.y1 = frame.color2.value().g / 255.0f; break;
                    case CurveYType::B2: bezier.y1 = frame.color2.value().b / 255.0f; break;
                    case CurveYType::V1: bezier.y1 = frame.value1; break;
                    case CurveYType::V2: bezier.y1 = frame.value2; break;
                    case CurveYType::V3: bezier.y1 = frame.value3; break;
                    case CurveYType::V4: bezier.y1 = frame.value4; break;
                    case CurveYType::V5: bezier.y1 = frame.value5; break;
                    case CurveYType::V6: bezier.y1 = frame.value6; break;
                    case CurveYType::ZeroOne: bezier.y1 = 0.0f; break;
                }
                bezier.x2 = timeline[i + 1].time;
                switch (curveYType) {
                    case CurveYType::R1: bezier.y2 = timeline[i + 1].color1.value().r / 255.0f; break;
                    case CurveYType::G1: bezier.y2 = timeline[i + 1].color1.value().g / 255.0f; break;
                    case CurveYType::B1: bezier.y2 = timeline[i + 1].color1.value().b / 255.0f; break;
                    case CurveYType::A1: bezier.y2 = timeline[i + 1].color1.value().a / 255.0f; break;
                    case CurveYType::R2: bezier.y2 = timeline[i + 1].color2.value().r / 255.0f; break;
                    case CurveYType::G2: bezier.y2 = timeline[i + 1].color2.value().g / 255.0f; break;
                    case CurveYType::B2: bezier.y2 = timeline[i + 1].color2.value().b / 255.0f; break;
                    case CurveYType::V1: bezier.y2 = timeline[i + 1].value1; break;
                    case CurveYType::V2: bezier.y2 = timeline[i + 1].value2; break;
                    case CurveYType::V3: bezier.y2 = timeline[i + 1].value3; break;
                    case CurveYType::V4: bezier.y2 = timeline[i + 1].value4; break;
                    case CurveYType::V5: bezier.y2 = timeline[i + 1].value5; break;
                    case CurveYType::V6: bezier.y2 = timeline[i + 1].value6; break;
                    case CurveYType::ZeroOne: bezier.y2 = 1.0f; break;
                }
                convertBezierCurve3xTo4x(bezier);
                frame.curve[bezierIndex * 4 + 0] = bezier.cx1;
                frame.curve[bezierIndex * 4 + 1] = bezier.cy1;
                frame.curve[bezierIndex * 4 + 2] = bezier.cx2;
                frame.curve[bezierIndex * 4 + 3] = bezier.cy2;
            }
        }
    }
}

void convertTimelineCurve4xTo3x(Timeline& timeline, CurveYType curveYType) {
    for (size_t i = 0; i + 1 < timeline.size(); i++) {
        auto& frame = timeline[i];
        if (frame.curveType == CurveType::CURVE_BEZIER) {
            size_t bezierIndex = CurveYTypeBezierIndex[(size_t)curveYType];
            BezierCurve bezier;
            bezier.cx1 = frame.curve[bezierIndex * 4 + 0];
            bezier.cy1 = frame.curve[bezierIndex * 4 + 1];
            bezier.cx2 = frame.curve[bezierIndex * 4 + 2];
            bezier.cy2 = frame.curve[bezierIndex * 4 + 3];
            bezier.x1 = frame.time;
            switch (curveYType) {
                case CurveYType::R1: bezier.y1 = frame.color1.value().r / 255.0f; break;
                case CurveYType::V1: bezier.y1 = frame.value1; break;
                case CurveYType::ZeroOne: bezier.y1 = 0.0f; break;
                default: bezier.y1 = 0.0f; break; // Other types are not handled in 4x to 3x conversion
            }
            bezier.x2 = timeline[i + 1].time;
            switch (curveYType) {
                case CurveYType::R1: bezier.y2 = timeline[i + 1].color1.value().r / 255.0f; break;
                case CurveYType::V1: bezier.y2 = timeline[i + 1].value1; break;
                case CurveYType::ZeroOne: bezier.y2 = 1.0f; break;
                default: bezier.y2 = 1.0f; break; // Other types are not handled in 4x to 3x conversion
            }
            convertBezierCurve4xTo3x(bezier);
            frame.curve[0] = bezier.cx1;
            frame.curve[1] = bezier.cy1;
            frame.curve[2] = bezier.cx2;
            frame.curve[3] = bezier.cy2;
        }
    }
}

void convertCurve3xTo4x(SkeletonData& skeleton) {
    for (auto& animation : skeleton.animations) {
        for (auto& [slotName, multiTimeline] : animation.slots)
            for (auto& [timelineType, timeline] : multiTimeline)
                if (timelineType == "rgba")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::R1, CurveYType::G1, CurveYType::B1, CurveYType::A1 });
                else if (timelineType == "rgba2")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::R1, CurveYType::G1, CurveYType::B1, CurveYType::A1, CurveYType::R2, CurveYType::G2, CurveYType::B2 });
        for (auto& [boneName, multiTimeline] : animation.bones)
            for (auto& [timelineType, timeline] : multiTimeline)
                if (timelineType == "rotate")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::V1 });
                else if (timelineType == "translate" || timelineType == "scale" || timelineType == "shear")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::V1, CurveYType::V2 });
        for (auto& [ikName, timeline] : animation.ik)
            convertTimelineCurve3xTo4x(timeline, { CurveYType::V1, CurveYType::V2 });
        for (auto& [transformName, timeline] : animation.transform)
            convertTimelineCurve3xTo4x(timeline, { CurveYType::V1, CurveYType::V2, CurveYType::V3, CurveYType::V4, CurveYType::V5, CurveYType::V6 });
        for (auto& [pathName, multiTimeline] : animation.path)
            for (auto& [timelineType, timeline] : multiTimeline)
                if (timelineType == "position" || timelineType == "spacing")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::V1 });
                else if (timelineType == "mix")
                    convertTimelineCurve3xTo4x(timeline, { CurveYType::V1, CurveYType::V2, CurveYType::V3 });
        for (auto& [skinName, skin] : animation.attachments)
            for (auto& [slotName, slot] : skin)
                for (auto& [attachmentName, multiTimeline] : slot)
                    for (auto& [timelineType, timeline] : multiTimeline)
                        if (timelineType == "deform")
                            convertTimelineCurve3xTo4x(timeline, { CurveYType::ZeroOne });
    }
}

TimelineFrame* findFrameAtTime(Timeline& timeline, float time) {
    for (auto& frame : timeline) {
        if (std::abs(frame.time - time) < 0.0001f) return &frame;
    }
    return nullptr;
}

const TimelineFrame* findFrameAtTime(const Timeline& timeline, float time) {
    for (const auto& frame : timeline) {
        if (std::abs(frame.time - time) < 0.0001f) return &frame;
    }
    return nullptr;
}

// Linear interpolation 
float interpolateValue(const Timeline& timeline, float targetTime) {
    if (timeline.empty()) return 0.0f;
    if (targetTime <= timeline.front().time) return timeline.front().value1;
    if (targetTime >= timeline.back().time) return timeline.back().value1;
    
    for (size_t i = 0; i + 1 < timeline.size(); i++) {
        const auto& frame1 = timeline[i];
        const auto& frame2 = timeline[i + 1];
        
        if (targetTime >= frame1.time && targetTime <= frame2.time) {
            float t = (targetTime - frame1.time) / (frame2.time - frame1.time);
            return frame1.value1 + t * (frame2.value1 - frame1.value1);
        }
    }
    return 0.0f;
}

// Bezier curve division function at parameter t (0-1)
// Input: base curve [x0, y0, x1, y1, x2, y2, x3, y3] , split point
// Output: two new curves
struct SplitCurveResult {
    std::vector<float> leftCurve;   // 4 value
    std::vector<float> rightCurve;  // 4 value
    float splitValue;               // value at split point
};

SplitCurveResult splitBezierCurve(const std::vector<float>& curve, float t) {
    // curve format: [cx1, cy1, cx2, cy2]
    // These are absolute control points.
    
    if (curve.size() < 4) {
        return {{}, {}, 0.0f};
    }
    
    float cx1 = curve[0], cy1 = curve[1];
    float cx2 = curve[2], cy2 = curve[3];
    
    // De Casteljau's algorithm
    // P0 = start point (from current frame)
    // P1 = (cx1, cy1)
    // P2 = (cx2, cy2)  
    // P3 = end point (from next frame)
    
    // Calculate the intermediate points
    // Q0 = P0 + t*(P1-P0)
    // Q1 = P1 + t*(P2-P1)
    // Q2 = P2 + t*(P3-P2)
    // R0 = Q0 + t*(Q1-Q0)
    // R1 = Q1 + t*(Q2-Q1)
    // S = R0 + t*(R1-R0) = split point
    
    // Left curve: P0 -> Q0 -> R0 -> S
    // Right curve: S -> R1 -> Q2 -> P3
    
    SplitCurveResult result;
    result.leftCurve.resize(4);
    result.rightCurve.resize(4);
    
    // Note: We need to know P0 and P3 to calculate accurately
    // but this function only handles control points.
    // so we only calculate relatively.
    
    // Left curve control points (relative to P0)
    result.leftCurve[0] = cx1;  // Will be adjusted later
    result.leftCurve[1] = cy1;
    result.leftCurve[2] = cx2;  // Will be adjusted later
    result.leftCurve[3] = cy2;
    
    // Right curve control points (relative to S)
    result.rightCurve[0] = cx1;  // Will be adjusted later
    result.rightCurve[1] = cy1;
    result.rightCurve[2] = cx2;
    result.rightCurve[3] = cy2;
    
    return result;
}

// split curve function
SplitCurveResult splitBezierCurveFull(float startTime, float startValue, 
                                      const std::vector<float>& curve,
                                      float endTime, float endValue,
                                      float splitTime) {
    SplitCurveResult result;
    
    if (curve.size() < 4 || splitTime <= startTime || splitTime >= endTime) {
        return result;
    }
    
    // Tính t parameter (0-1)
    float t = (splitTime - startTime) / (endTime - startTime);
    
    // Bezier curve points
    float x0 = startTime, y0 = startValue;
    float x1 = curve[0], y1 = curve[1];
    float x2 = curve[2], y2 = curve[3];
    float x3 = endTime, y3 = endValue;
    
    // De Casteljau's algorithm
    // First level
    float q0x = x0 + t * (x1 - x0);
    float q0y = y0 + t * (y1 - y0);
    float q1x = x1 + t * (x2 - x1);
    float q1y = y1 + t * (y2 - y1);
    float q2x = x2 + t * (x3 - x2);
    float q2y = y2 + t * (y3 - y2);
    
    // Second level
    float r0x = q0x + t * (q1x - q0x);
    float r0y = q0y + t * (q1y - q0y);
    float r1x = q1x + t * (q2x - q1x);
    float r1y = q1y + t * (q2y - q1y);
    
    // Split point
    float sx = r0x + t * (r1x - r0x);
    float sy = r0y + t * (r1y - r0y);
    
    // Left curve: (x0,y0) -> (q0x,q0y) -> (r0x,r0y) -> (sx,sy)
    result.leftCurve = {q0x, q0y, r0x, r0y};
    
    // Right curve: (sx,sy) -> (r1x,r1y) -> (q2x,q2y) -> (x3,y3)
    result.rightCurve = {r1x, r1y, q2x, q2y};
    
    result.splitValue = sy;
    
    return result;
}

// =====================================================
// Function to split timeline at breakpointsv
// =====================================================

void splitTimelineAtBreakpoints(Timeline& timeline, const std::set<float>& breakpoints) {
    if (timeline.empty() || breakpoints.empty()) return;
    
    Timeline newTimeline;
    
    for (size_t i = 0; i < timeline.size(); i++) {
        const auto& frame = timeline[i];
        newTimeline.push_back(frame);
        
        // Check if there is a breakpoint between this frame and the next frame
        if (i + 1 < timeline.size() && frame.curveType == CurveType::CURVE_BEZIER) {
            const auto& nextFrame = timeline[i + 1];
            
            // Find all breakpoints between frame.time and nextFrame.time
            std::vector<float> splits;
            for (float bp : breakpoints) {
                if (bp > frame.time && bp < nextFrame.time) {
                    splits.push_back(bp);
                }
            }
            
            if (!splits.empty() && frame.curve.size() >= 4) {
                // Sort breakpoints
                std::sort(splits.begin(), splits.end());
                
                float currentTime = frame.time;
                float currentValue = frame.value1;
                std::vector<float> currentCurve = frame.curve;
                
                // Split at each breakpoint
                for (float splitTime : splits) {
                    auto splitResult = splitBezierCurveFull(
                        currentTime, currentValue,
                        currentCurve,
                        nextFrame.time, nextFrame.value1,
                        splitTime
                    );
                    
                    // Update curve of the last frame in newTimeline
                    newTimeline.back().curve = splitResult.leftCurve;
                    
                    // Create a new frame at the split point
                    TimelineFrame splitFrame;
                    splitFrame.time = splitTime;
                    splitFrame.value1 = splitResult.splitValue;
                    splitFrame.curveType = CurveType::CURVE_BEZIER;
                    splitFrame.curve = splitResult.rightCurve;
                    newTimeline.push_back(splitFrame);
                    
                    // Update for next split
                    currentTime = splitTime;
                    currentValue = splitResult.splitValue;
                    currentCurve = splitResult.rightCurve;
                }
            }
        }
    }
    
    timeline = newTimeline;
}

// =====================================================
// merge  function
// =====================================================

void mergeTranslateXYToTranslate(SkeletonData& skeleton) {
    for (auto& animation : skeleton.animations) {
        //Translate
        for (auto& [boneName, boneTimeline] : animation.bones) {
            bool hasTranslateX = boneTimeline.contains("translatex");
            bool hasTranslateY = boneTimeline.contains("translatey");
            
            if (!hasTranslateX && !hasTranslateY) continue;
            
            // Create copies for processing
            Timeline tlX = hasTranslateX ? boneTimeline["translatex"] : Timeline();
            Timeline tlY = hasTranslateY ? boneTimeline["translatey"] : Timeline();
            
            // Collect all time points
            std::set<float> allTimes;
            if (hasTranslateX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasTranslateY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Split curves at breakpoints
            if (hasTranslateX) splitTimelineAtBreakpoints(tlX, allTimes);
            if (hasTranslateY) splitTimelineAtBreakpoints(tlY, allTimes);
            
            // Collect times after split
            allTimes.clear();
            if (hasTranslateX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasTranslateY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Merge timelines
            Timeline mergedTimeline;
            
            for (float time : allTimes) {
                TimelineFrame newFrame;
                newFrame.time = time;
                
                const TimelineFrame* frameX = hasTranslateX ? findFrameAtTime(tlX, time) : nullptr;
                const TimelineFrame* frameY = hasTranslateY ? findFrameAtTime(tlY, time) : nullptr;
                
                // value
                newFrame.value1 = frameX ? frameX->value1 : 0.0f;
                newFrame.value2 = frameY ? frameY->value1 : 0.0f;
                
                // Curve
                bool hasXCurve = frameX && frameX->curveType == CurveType::CURVE_BEZIER;
                bool hasYCurve = frameY && frameY->curveType == CurveType::CURVE_BEZIER;
                
                if (hasXCurve || hasYCurve) {
                    newFrame.curveType = CurveType::CURVE_BEZIER;
                    newFrame.curve.resize(8, 0.0f);
                    
                    // Copy curve X
                    if (hasXCurve && frameX->curve.size() >= 4) {
                        newFrame.curve[0] = frameX->curve[0];
                        newFrame.curve[1] = frameX->curve[1];
                        newFrame.curve[2] = frameX->curve[2];
                        newFrame.curve[3] = frameX->curve[3];
                    }
                    
                    // Copy curve Y
                    if (hasYCurve && frameY->curve.size() >= 4) {
                        newFrame.curve[4] = frameY->curve[0];
                        newFrame.curve[5] = frameY->curve[1];
                        newFrame.curve[6] = frameY->curve[2];
                        newFrame.curve[7] = frameY->curve[3];
                    }
                } else if ((frameX && frameX->curveType == CurveType::CURVE_STEPPED) ||
                          (frameY && frameY->curveType == CurveType::CURVE_STEPPED)) {
                    newFrame.curveType = CurveType::CURVE_STEPPED;
                } else {
                    newFrame.curveType = CurveType::CURVE_LINEAR;
                }
                
                mergedTimeline.push_back(newFrame);
            }
            
            boneTimeline["translate"] = mergedTimeline;
            if (hasTranslateX) boneTimeline.erase("translatex");
            if (hasTranslateY) boneTimeline.erase("translatey");
        }
        //Scale
        for (auto& [boneName, boneTimeline] : animation.bones) {
            bool hasScaleX = boneTimeline.contains("scalex");
            bool hasScaleY = boneTimeline.contains("scaley");
            
            if (!hasScaleX && !hasScaleY) continue;
            
            // Create copies for processing
            Timeline tlX = hasScaleX ? boneTimeline["scalex"] : Timeline();
            Timeline tlY = hasScaleY ? boneTimeline["scaley"] : Timeline();
            
            // Collect all time points
            std::set<float> allTimes;
            if (hasScaleX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasScaleY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Split curves at breakpoints
            if (hasScaleX) splitTimelineAtBreakpoints(tlX, allTimes);
            if (hasScaleY) splitTimelineAtBreakpoints(tlY, allTimes);
            
            // Collect times after split
            allTimes.clear();
            if (hasScaleX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasScaleY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Merge timelines
            Timeline mergedTimeline;
            
            for (float time : allTimes) {
                TimelineFrame newFrame;
                newFrame.time = time;
                
                const TimelineFrame* frameX = hasScaleX ? findFrameAtTime(tlX, time) : nullptr;
                const TimelineFrame* frameY = hasScaleY ? findFrameAtTime(tlY, time) : nullptr;
                
                // value
                newFrame.value1 = frameX ? frameX->value1 : 1.0f;
                newFrame.value2 = frameY ? frameY->value1 : 1.0f;
                
                // Curve
                bool hasXCurve = frameX && frameX->curveType == CurveType::CURVE_BEZIER;
                bool hasYCurve = frameY && frameY->curveType == CurveType::CURVE_BEZIER;
                
                if (hasXCurve || hasYCurve) {
                    newFrame.curveType = CurveType::CURVE_BEZIER;
                    newFrame.curve.resize(8, 0.0f);
                    
                    // Copy curve X
                    if (hasXCurve && frameX->curve.size() >= 4) {
                        newFrame.curve[0] = frameX->curve[0];
                        newFrame.curve[1] = frameX->curve[1];
                        newFrame.curve[2] = frameX->curve[2];
                        newFrame.curve[3] = frameX->curve[3];
                    }
                    
                    // Copy curve Y
                    if (hasYCurve && frameY->curve.size() >= 4) {
                        newFrame.curve[4] = frameY->curve[0];
                        newFrame.curve[5] = frameY->curve[1];
                        newFrame.curve[6] = frameY->curve[2];
                        newFrame.curve[7] = frameY->curve[3];
                    }
                } else if ((frameX && frameX->curveType == CurveType::CURVE_STEPPED) ||
                          (frameY && frameY->curveType == CurveType::CURVE_STEPPED)) {
                    newFrame.curveType = CurveType::CURVE_STEPPED;
                } else {
                    newFrame.curveType = CurveType::CURVE_LINEAR;
                }
                
                mergedTimeline.push_back(newFrame);
            }
            
            boneTimeline["scale"] = mergedTimeline;
            if (hasScaleX) boneTimeline.erase("scalex");
            if (hasScaleY) boneTimeline.erase("scaley");
        }
        //Shear
        for (auto& [boneName, boneTimeline] : animation.bones) {
            bool hasShearX = boneTimeline.contains("shearx");
            bool hasShearY = boneTimeline.contains("sheary");
            
            if (!hasShearX && !hasShearY) continue;
            
            // Create copies for processing
            Timeline tlX = hasShearX ? boneTimeline["shearx"] : Timeline();
            Timeline tlY = hasShearY ? boneTimeline["sheary"] : Timeline();
            
            // Collect all time points
            std::set<float> allTimes;
            if (hasShearX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasShearY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Split curves at breakpoints
            if (hasShearX) splitTimelineAtBreakpoints(tlX, allTimes);
            if (hasShearY) splitTimelineAtBreakpoints(tlY, allTimes);
            
            // Collect times after split
            allTimes.clear();
            if (hasShearX) for (const auto& f : tlX) allTimes.insert(f.time);
            if (hasShearY) for (const auto& f : tlY) allTimes.insert(f.time);
            
            // Merge timelines
            Timeline mergedTimeline;
            
            for (float time : allTimes) {
                TimelineFrame newFrame;
                newFrame.time = time;
                
                const TimelineFrame* frameX = hasShearX ? findFrameAtTime(tlX, time) : nullptr;
                const TimelineFrame* frameY = hasShearY ? findFrameAtTime(tlY, time) : nullptr;
                
                // value
                newFrame.value1 = frameX ? frameX->value1 : 0.0f;
                newFrame.value2 = frameY ? frameY->value1 : 0.0f;
                
                // Curve
                bool hasXCurve = frameX && frameX->curveType == CurveType::CURVE_BEZIER;
                bool hasYCurve = frameY && frameY->curveType == CurveType::CURVE_BEZIER;
                
                if (hasXCurve || hasYCurve) {
                    newFrame.curveType = CurveType::CURVE_BEZIER;
                    newFrame.curve.resize(8, 0.0f);
                    
                    // Copy curve X
                    if (hasXCurve && frameX->curve.size() >= 4) {
                        newFrame.curve[0] = frameX->curve[0];
                        newFrame.curve[1] = frameX->curve[1];
                        newFrame.curve[2] = frameX->curve[2];
                        newFrame.curve[3] = frameX->curve[3];
                    }
                    
                    // Copy curve Y
                    if (hasYCurve && frameY->curve.size() >= 4) {
                        newFrame.curve[4] = frameY->curve[0];
                        newFrame.curve[5] = frameY->curve[1];
                        newFrame.curve[6] = frameY->curve[2];
                        newFrame.curve[7] = frameY->curve[3];
                    }
                } else if ((frameX && frameX->curveType == CurveType::CURVE_STEPPED) ||
                          (frameY && frameY->curveType == CurveType::CURVE_STEPPED)) {
                    newFrame.curveType = CurveType::CURVE_STEPPED;
                } else {
                    newFrame.curveType = CurveType::CURVE_LINEAR;
                }
                
                mergedTimeline.push_back(newFrame);
            }
            
            boneTimeline["shear"] = mergedTimeline;
            if (hasShearX) boneTimeline.erase("shearx");
            if (hasShearY) boneTimeline.erase("sheary");
        }
 
    }
}

void convertCurve4xTo3x(SkeletonData& skeleton) {
    for (auto& animation : skeleton.animations) {
        for (auto& [slotName, multiTimeline] : animation.slots)
            for (auto& [timelineType, timeline] : multiTimeline)
                if (timelineType == "rgba" || timelineType == "rgba2" || timelineType == "rgb" || timelineType == "rgb2")
                    convertTimelineCurve4xTo3x(timeline, CurveYType::R1);
        for (auto& [boneName, multiTimeline] : animation.bones)
            for (auto& [timelineType, timeline] : multiTimeline)
                convertTimelineCurve4xTo3x(timeline, CurveYType::V1);
        for (auto& [ikName, timeline] : animation.ik)
            convertTimelineCurve4xTo3x(timeline, CurveYType::V1);
        for (auto& [transformName, timeline] : animation.transform)
            convertTimelineCurve4xTo3x(timeline, CurveYType::V1);
        for (auto& [pathName, multiTimeline] : animation.path)
            for (auto& [timelineType, timeline] : multiTimeline)
                convertTimelineCurve4xTo3x(timeline, CurveYType::V1);
        for (auto& [skinName, skin] : animation.attachments)
            for (auto& [slotName, slot] : skin)
                for (auto& [attachmentName, multiTimeline] : slot)
                    for (auto& [timelineType, timeline] : multiTimeline)
                        if (timelineType == "deform")
                            convertTimelineCurve4xTo3x(timeline, CurveYType::ZeroOne);
    }
}

void removeCurve(SkeletonData& skeleton) {
    for (auto& animation : skeleton.animations) {
        for (auto& [slotName, multiTimeline] : animation.slots)
            for (auto& [timelineType, timeline] : multiTimeline)
                for (auto& frame : timeline)
                    frame.curveType = CurveType::CURVE_LINEAR;
        for (auto& [boneName, multiTimeline] : animation.bones)
            for (auto& [timelineType, timeline] : multiTimeline)
                for (auto& frame : timeline)
                    frame.curveType = CurveType::CURVE_LINEAR;
        for (auto& [ikName, timeline] : animation.ik)
            for (auto& frame : timeline)
                frame.curveType = CurveType::CURVE_LINEAR;
        for (auto& [transformName, timeline] : animation.transform)
            for (auto& frame : timeline)
                frame.curveType = CurveType::CURVE_LINEAR;
        for (auto& [pathName, multiTimeline] : animation.path)
            for (auto& [timelineType, timeline] : multiTimeline)
                for (auto& frame : timeline)
                    frame.curveType = CurveType::CURVE_LINEAR;
        for (auto& [skinName, skin] : animation.attachments)
            for (auto& [slotName, slot] : skin)
                for (auto& [attachmentName, multiTimeline] : slot)
                    for (auto& [timelineType, timeline] : multiTimeline)
                        for (auto& frame : timeline)
                            frame.curveType = CurveType::CURVE_LINEAR;
    }
}
