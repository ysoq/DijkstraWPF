using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Shapes;
using Dijkstra.NET.Graph;
using Dijkstra.NET.Graph.Simple;
using Dijkstra.NET.ShortestPath;

namespace DijkstraWPF
{
    // 点结构
    // 点结构
    public struct Point : IEquatable<Point>
    {
        public double X { get; }
        public double Y { get; }

        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }

        public bool Equals(Point other)
        {
            return Math.Abs(X - other.X) < 1e-10 && Math.Abs(Y - other.Y) < 1e-10;
        }

        public override bool Equals(object obj) => obj is Point other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(X, Y);
        public override string ToString() => $"({X:F2}, {Y:F2})";

        public double DistanceTo(Point other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }

    // 线段结构
    public struct LineSegment : IEquatable<LineSegment>
    {
        public Point Start { get; }
        public Point End { get; }

        public int Id => GetHashCode();

        public LineSegment(Point start, Point end)
        {
            Start = start;
            End = end;
        }

        public bool Equals(LineSegment other)
        {
            return (Start.Equals(other.Start) && End.Equals(other.End)) ||
                   (Start.Equals(other.End) && End.Equals(other.Start));
        }

        public override bool Equals(object obj) => obj is LineSegment other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Start, End);

        public override string ToString() => $"{Start} -> {End}";

        // 计算线段长度
        public int Length => (int)(Math.Sqrt(Math.Pow(End.X - Start.X, 2) + Math.Pow(End.Y - Start.Y, 2)));

        // 计算点到线段的垂直交点
        public (Point projection, double t, bool onSegment) GetProjection(Point point)
        {
            double A = point.X - Start.X;
            double B = point.Y - Start.Y;
            double C = End.X - Start.X;
            double D = End.Y - Start.Y;

            double dot = A * C + B * D;
            double lenSq = C * C + D * D;
            double t = lenSq != 0 ? dot / lenSq : -1;

            Point projection;
            bool onSegment = t >= 0 && t <= 1;

            if (t < 0)
            {
                projection = Start;
            }
            else if (t > 1)
            {
                projection = End;
            }
            else
            {
                projection = new Point(
                    Start.X + t * C,
                    Start.Y + t * D
                );
            }

            return (projection, t, onSegment);
        }
    }


    public class LineSegmentProcessor
    {
        private const double Epsilon = 1e-10;

        // 主方法：处理相交线段并截断
        public List<LineSegment> ProcessIntersectingSegments(List<LineSegment> segments)
        {
            if (segments == null || segments.Count == 0)
                return new List<LineSegment>();

            // 收集所有需要分割的点
            var splitPoints = new Dictionary<LineSegment, List<Point>>();

            // 初始化字典，为每条线段添加起点和终点
            foreach (var segment in segments)
            {
                splitPoints[segment] = new List<Point> { segment.Start, segment.End };
            }

            // 找出所有交点
            for (int i = 0; i < segments.Count; i++)
            {
                for (int j = i + 1; j < segments.Count; j++)
                {
                    var seg1 = segments[i];
                    var seg2 = segments[j];

                    if (TryFindIntersection(seg1, seg2, out Point intersection))
                    {
                        // 如果交点在seg1上（不在端点处），添加到分割点列表
                        if (IsPointOnSegment(intersection, seg1) &&
                            !intersection.Equals(seg1.Start) &&
                            !intersection.Equals(seg1.End))
                        {
                            splitPoints[seg1].Add(intersection);
                        }

                        // 如果交点在seg2上（不在端点处），添加到分割点列表
                        if (IsPointOnSegment(intersection, seg2) &&
                            !intersection.Equals(seg2.Start) &&
                            !intersection.Equals(seg2.End))
                        {
                            splitPoints[seg2].Add(intersection);
                        }
                    }
                }
            }

            // 分割每条线段
            var result = new List<LineSegment>();

            foreach (var kvp in splitPoints)
            {
                var segment = kvp.Key;
                var points = kvp.Value;

                // 去重并排序
                var uniquePoints = points.Distinct().ToList();

                // 沿着线段方向排序点
                SortPointsAlongSegment(segment, uniquePoints);

                // 创建新的线段
                for (int i = 0; i < uniquePoints.Count - 1; i++)
                {
                    var newSegment = new LineSegment(uniquePoints[i], uniquePoints[i + 1]);

                    // 跳过长度为零的线段
                    if (!newSegment.Start.Equals(newSegment.End))
                    {
                        result.Add(newSegment);
                    }
                }
            }

            // 去除重复的线段
            return result.Distinct().ToList();
        }

        // 判断两条线段是否相交并计算交点
        private bool TryFindIntersection(LineSegment seg1, LineSegment seg2, out Point intersection)
        {
            intersection = new Point(double.NaN, double.NaN);

            var p1 = seg1.Start;
            var p2 = seg1.End;
            var p3 = seg2.Start;
            var p4 = seg2.End;

            // 计算线段方向
            var d1 = new Point(p2.X - p1.X, p2.Y - p1.Y);
            var d2 = new Point(p4.X - p3.X, p4.Y - p3.Y);

            // 计算行列式
            double det = d1.X * d2.Y - d1.Y * d2.X;

            // 如果平行或共线
            if (Math.Abs(det) < Epsilon)
            {
                // 处理共线情况
                if (AreCollinear(p1, p2, p3, p4))
                {
                    // 在共线情况下，寻找重叠部分的交点
                    // 这里我们只返回一个交点，实际可能有多个
                    return TryFindCollinearIntersection(p1, p2, p3, p4, out intersection);
                }
                return false;
            }

            // 计算参数
            double t = ((p3.X - p1.X) * d2.Y - (p3.Y - p1.Y) * d2.X) / det;
            double u = ((p3.X - p1.X) * d1.Y - (p3.Y - p1.Y) * d1.X) / det;

            // 检查交点是否在线段上
            if (t >= -Epsilon && t <= 1 + Epsilon && u >= -Epsilon && u <= 1 + Epsilon)
            {
                // 计算交点坐标
                double x = p1.X + t * d1.X;
                double y = p1.Y + t * d1.Y;

                intersection = new Point(x, y);
                return true;
            }

            return false;
        }

        // 判断点是否在线段上
        private bool IsPointOnSegment(Point point, LineSegment segment)
        {
            return IsPointOnLine(point, segment.Start, segment.End) &&
                   IsBetween(point, segment.Start, segment.End);
        }

        // 判断点是否在直线上
        private bool IsPointOnLine(Point point, Point lineStart, Point lineEnd)
        {
            double crossProduct = (point.Y - lineStart.Y) * (lineEnd.X - lineStart.X) -
                                (point.X - lineStart.X) * (lineEnd.Y - lineStart.Y);

            return Math.Abs(crossProduct) < Epsilon;
        }

        // 判断点是否在两个点之间
        private bool IsBetween(Point point, Point a, Point b)
        {
            // 检查点是否在x和y方向上都在a和b之间
            bool inX = (point.X >= Math.Min(a.X, b.X) - Epsilon) &&
                      (point.X <= Math.Max(a.X, b.X) + Epsilon);
            bool inY = (point.Y >= Math.Min(a.Y, b.Y) - Epsilon) &&
                      (point.Y <= Math.Max(a.Y, b.Y) + Epsilon);

            return inX && inY;
        }

        // 判断四点是否共线
        private bool AreCollinear(Point p1, Point p2, Point p3, Point p4)
        {
            double cross1 = (p2.Y - p1.Y) * (p3.X - p2.X) - (p2.X - p1.X) * (p3.Y - p2.Y);
            double cross2 = (p2.Y - p1.Y) * (p4.X - p2.X) - (p2.X - p1.X) * (p4.Y - p2.Y);

            return Math.Abs(cross1) < Epsilon && Math.Abs(cross2) < Epsilon;
        }

        // 查找共线情况下的交点
        private bool TryFindCollinearIntersection(Point p1, Point p2, Point p3, Point p4, out Point intersection)
        {
            intersection = new Point(double.NaN, double.NaN);

            // 计算四个点的最小和最大坐标
            double minX1 = Math.Min(p1.X, p2.X);
            double maxX1 = Math.Max(p1.X, p2.X);
            double minY1 = Math.Min(p1.Y, p2.Y);
            double maxY1 = Math.Max(p1.Y, p2.Y);

            double minX2 = Math.Min(p3.X, p4.X);
            double maxX2 = Math.Max(p3.X, p4.X);
            double minY2 = Math.Min(p3.Y, p4.Y);
            double maxY2 = Math.Max(p3.Y, p4.Y);

            // 检查是否有重叠
            bool overlapX = maxX1 >= minX2 && maxX2 >= minX1;
            bool overlapY = maxY1 >= minY2 && maxY2 >= minY1;

            if (!overlapX || !overlapY)
                return false;

            // 计算重叠区域的中心点作为交点
            double intersectMinX = Math.Max(minX1, minX2);
            double intersectMaxX = Math.Min(maxX1, maxX2);
            double intersectMinY = Math.Max(minY1, minY2);
            double intersectMaxY = Math.Min(maxY1, maxY2);

            double centerX = (intersectMinX + intersectMaxX) / 2;
            double centerY = (intersectMinY + intersectMaxY) / 2;

            intersection = new Point(centerX, centerY);
            return true;
        }

        // 沿着线段方向排序点
        private void SortPointsAlongSegment(LineSegment segment, List<Point> points)
        {
            // 使用线段的参数t值进行排序
            points.Sort((p1, p2) =>
            {
                double t1 = GetParameterAlongSegment(segment, p1);
                double t2 = GetParameterAlongSegment(segment, p2);
                return t1.CompareTo(t2);
            });
        }

        // 获取点在线段上的参数t值
        private double GetParameterAlongSegment(LineSegment segment, Point point)
        {
            var start = segment.Start;
            var end = segment.End;

            // 如果线段在x方向更长，使用x坐标
            if (Math.Abs(end.X - start.X) > Math.Abs(end.Y - start.Y))
            {
                if (Math.Abs(end.X - start.X) < Epsilon)
                    return 0;
                return (point.X - start.X) / (end.X - start.X);
            }
            else // 否则使用y坐标
            {
                if (Math.Abs(end.Y - start.Y) < Epsilon)
                    return 0;
                return (point.Y - start.Y) / (end.Y - start.Y);
            }
        }
    }

    

    // 图构建器
    public class GraphBuilder
    {
        private readonly Graph<int, string> graph;
        private Dictionary<int, uint> nodeIds = new Dictionary<int, uint> ();
        private Dictionary<uint, Point> idByNode = new Dictionary<uint, Point> ();
        public GraphBuilder()
        {
           this.graph = new Graph<int, string>();
        }
        public void Set(List<LineSegment> lines)
        {
            foreach (var item in lines)
            {
                AddNode(item.Start);
                AddNode(item.End);
            }

            foreach (var line in lines)
            {
                var endList = lines.Where(x=> x.Start.Equals(line.End));
                graph.Connect(GetNodeId(line.Start), GetNodeId(line.End), line.Length, "");

                foreach (var item in endList)
                {
                    graph.Connect(GetNodeId(line.End), GetNodeId(item.Start), item.Length, ""); 
                }
            }
        }

        public List<Point> FindShortestPath(Point start, Point end)
        {
            var startId = GetNodeId(start);
            var endId = GetNodeId(end);
            ShortestPathResult result = graph.Dijkstra(startId, endId);

            IEnumerable<uint> path = result.GetPath();
            return path.Select(x => idByNode[x]).ToList();
        }

        public List<LineSegment>? AddTemporaryPoint(List<LineSegment> lines, Point point)
        {
            // 找到最近的线段和投影点
            var (projection, nearestSegment ) = FindNearestSegment(lines, point);
            if(projection == null || nearestSegment == null)
            {
                return null;
            }
            if (nearestSegment.Value.Start.Equals( projection ) ||
                nearestSegment.Value.End.Equals(projection))
            {
                return new List<LineSegment>() { nearestSegment.Value  };
            } else
            {
                return new List<LineSegment>()
                {
                     new LineSegment (nearestSegment.Value.Start, projection.Value),
                     new LineSegment (projection.Value, nearestSegment.Value.End),
                     new LineSegment(point, projection.Value)
                };
            }
        }

        private (Point?, LineSegment?) FindNearestSegment(List<LineSegment> lines, Point point)
        {
            LineSegment? nearestSegment = null;
            Point? nearestProjection = null;
            double nearestDistance = double.MaxValue;

            foreach (var segment in lines)
            {
                var (projection, t, onSegment) = segment.GetProjection(point);
                double distance = point.DistanceTo(projection);

                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestProjection = projection;
                    nearestSegment = segment;
                }
            }

            return (nearestProjection, nearestSegment);
        }

        private void AddNode(Point point)
        {
            if (!nodeIds.ContainsKey(point.GetHashCode()))
            {
                var id = graph.AddNode(point.GetHashCode());
                nodeIds[point.GetHashCode()] = id;
                idByNode[id] = point;
            }
        }

        private uint GetNodeId(Point point)
        {
            return nodeIds[point.GetHashCode()]; 
        }

    }
}
