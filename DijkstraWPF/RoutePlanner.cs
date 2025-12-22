using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Dijkstra.NET.Graph;
using Dijkstra.NET.ShortestPath;

namespace DijkstraWPF
{
    public class RoutePlanner
    {
        private List<LineSegment> _lines;
        private Dictionary<Point2D, uint> _nodeIds;
        private Dictionary<uint, Point2D> _idToPoint;
        private Graph<Point2D, int> _graph;
        private uint _nextId;

        public RoutePlanner(List<LineSegment> lines)
        {
            _lines = lines;
            _nodeIds = new Dictionary<Point2D, uint>();
            _idToPoint = new Dictionary<uint, Point2D>();
            _graph = new Graph<Point2D, int>();
            _nextId = 1;

            BuildGraph();
        }

        private uint AddNode(Point2D point)
        {
            if (!_nodeIds.ContainsKey(point))
            {
                _nodeIds[point] = _nextId;
                _idToPoint[_nextId] = point;
                _graph.AddNode(point);
                _nextId++;
            }
            return _nodeIds[point];
        }

        private void BuildGraph()
        {
            // 添加所有线路的起点和终点作为节点
            foreach (var line in _lines)
            {
                var startId = AddNode(line.Start);
                var endId = AddNode(line.End);

                // 添加双向边
                int weight = (int)Math.Round(line.Length * 100); // 放大权重以保持精度
                _graph.Connect(startId, endId, weight, 0);
                _graph.Connect(endId, startId, weight, 0);
            }
        }

        public (List<Point2D> path, List<Point2D> connectionPoints) FindShortestPath(
            Point2D startPoint, Point2D endPoint)
        {
            // 1. 查找起点到最近线路的连接点
            var startConnection = FindConnectionPoint(startPoint);

            // 2. 查找终点到最近线路的连接点
            var endConnection = FindConnectionPoint(endPoint);

            // 3. 添加临时连接点到图
            uint startNodeId = 0;
            uint endNodeId = 0;

            if (startConnection.isOnSegment)
            {
                startNodeId = AddNode(startConnection.projection);
                ConnectToLine(startConnection, startNodeId);
            }
            else
            {
                // 连接到最近的端点
                startNodeId = _nodeIds[startConnection.nearestEndpoint];
                var tempId = AddNode(startPoint);
                int weight = (int)Math.Round(startPoint.DistanceTo(startConnection.nearestEndpoint) * 100);
                _graph.Connect(tempId, startNodeId, weight, 0);
                _graph.Connect(startNodeId, tempId, weight, 0);
                startNodeId = tempId;
            }

            if (endConnection.isOnSegment)
            {
                endNodeId = AddNode(endConnection.projection);
                ConnectToLine(endConnection, endNodeId);
            }
            else
            {
                endNodeId = _nodeIds[endConnection.nearestEndpoint];
                var tempId = AddNode(endPoint);
                int weight = (int)Math.Round(endPoint.DistanceTo(endConnection.nearestEndpoint) * 100);
                _graph.Connect(tempId, endNodeId, weight, 0);
                _graph.Connect(endNodeId, tempId, weight, 0);
                endNodeId = tempId;
            }

            // 4. 使用Dijkstra算法查找最短路径
            var result = _graph.Dijkstra(startNodeId, endNodeId);

            // 5. 提取路径点
            var pathPoints = new List<Point2D>();
            if (result.IsFounded)
            {
                foreach (var nodeId in result.GetPath())
                {
                    pathPoints.Add(_idToPoint[nodeId]);
                }
            }

            var connectionPoints = new List<Point2D>
        {
            startConnection.isOnSegment ? startConnection.projection : startPoint,
            endConnection.isOnSegment ? endConnection.projection : endPoint
        };

            return (pathPoints, connectionPoints);
        }

        private (Point2D projection, bool isOnSegment, Point2D nearestEndpoint, LineSegment line)
            FindConnectionPoint(Point2D point)
        {
            Point2D bestProjection = null;
            Point2D nearestEndpoint = null;
            LineSegment bestLine = null;
            double minDistance = double.MaxValue;
            bool isOnSegment = false;

            // 首先查找垂直线段上的投影点
            foreach (var line in _lines)
            {
                var (projection, onSegment) = line.GetProjection(point);
                double distance = point.DistanceTo(projection);

                if (onSegment && distance < minDistance)
                {
                    minDistance = distance;
                    bestProjection = projection;
                    bestLine = line;
                    isOnSegment = true;
                }
            }

            // 如果没有找到在线段上的投影，查找最近的端点
            if (!isOnSegment)
            {
                minDistance = double.MaxValue;
                foreach (var line in _lines)
                {
                    double distToStart = point.DistanceTo(line.Start);
                    double distToEnd = point.DistanceTo(line.End);

                    if (distToStart < minDistance)
                    {
                        minDistance = distToStart;
                        bestProjection = line.Start;
                        nearestEndpoint = line.Start;
                        bestLine = line;
                    }

                    if (distToEnd < minDistance)
                    {
                        minDistance = distToEnd;
                        bestProjection = line.End;
                        nearestEndpoint = line.End;
                        bestLine = line;
                    }
                }
            }

            return (bestProjection, isOnSegment, nearestEndpoint, bestLine);
        }

        private void ConnectToLine(
            (Point2D projection, bool isOnSegment, Point2D nearestEndpoint, LineSegment line) connection,
            uint projectionNodeId)
        {
            var line = connection.line;
            var projection = connection.projection;

            // 移除原有的边
            var startId = _nodeIds[line.Start];
            var endId = _nodeIds[line.End];

            // 重新添加边：起点->投影点，投影点->终点
            int weight1 = (int)Math.Round(line.Start.DistanceTo(projection) * 100);
            int weight2 = (int)Math.Round(projection.DistanceTo(line.End) * 100);

            _graph.Connect(startId, projectionNodeId, weight1, 0);
            _graph.Connect(projectionNodeId, startId, weight1, 0);

            _graph.Connect(projectionNodeId, endId, weight2, 0);
            _graph.Connect(endId, projectionNodeId, weight2, 0);
        }
    }

}
