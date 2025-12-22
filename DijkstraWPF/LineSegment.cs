using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DijkstraWPF
{
    public class LineSegment
    {
        public Point2D Start { get; set; }
        public Point2D End { get; set; }

        public LineSegment(Point2D start, Point2D end)
        {
            Start = start;
            End = end;
        }

        public double Length => Start.DistanceTo(End);

        // 计算点到线段的垂直交点
        public (Point2D projection, bool onSegment) GetProjection(Point2D point)
        {
            double A = point.X - Start.X;
            double B = point.Y - Start.Y;
            double C = End.X - Start.X;
            double D = End.Y - Start.Y;

            double dot = A * C + B * D;
            double lenSq = C * C + D * D;
            double param = lenSq != 0 ? dot / lenSq : -1;

            Point2D projection;
            if (param < 0)
            {
                projection = Start;
            }
            else if (param > 1)
            {
                projection = End;
            }
            else
            {
                projection = new Point2D(
                    Start.X + param * C,
                    Start.Y + param * D
                );
            }

            return (projection, param >= 0 && param <= 1);
        }
    }
}
