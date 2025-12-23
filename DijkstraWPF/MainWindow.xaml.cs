using System.Collections.Generic;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace DijkstraWPF;

/// <summary>
/// Interaction logic for MainWindow.xaml
/// </summary>
public partial class MainWindow : Window
{
    private List<LineSegment> _lines;
    private Point? _startPoint;
    private Point? _endPoint;
    private GraphBuilder _planner;
    private bool _isSelectingStart = true;
    private List<UIElement> _pathElements = new List<UIElement>();
    private UIElement _startMarker;
    private UIElement _endMarker;

    public MainWindow()
    {
        InitializeComponent();
        InitializeData();
        DrawMap();
        UpdateStatus();
    }

    private List<LineSegment> GetLines()
    {
        var lines = new List<LineSegment>
        {
            new LineSegment(new Point(196.5, 73.84375), new Point(189.5, 374.84375)),
            new LineSegment(new Point(133.5, 338.84375), new Point(441.5, 206.84375)),
            new LineSegment(new Point(397.5, 123.84375), new Point(363.5, 367.84375)),
            new LineSegment(new Point(70.5, 181.84375), new Point(463.5, 123.84375)),
            new LineSegment(new Point(302.5, 85.84375), new Point(257.5, 448.84375)),
            new LineSegment(new Point(107.5, 267.84375), new Point(559.5, 141.84375))
        };

        var processor = new LineSegmentProcessor();
        var result1 = processor.ProcessIntersectingSegments(lines);

        return result1;
    }

    private void InitializeData()
    {
        // 初始化示例数据
        var result1 = GetLines();

        _lines = result1;
        
        // 初始化图构建器
        _planner = new GraphBuilder();
        _planner.Set(_lines);
    }

    private void DrawMap()
    {
        canvas.Children.Clear();

        // 绘制地图线路
        foreach (var line in _lines)
        {
            var roadLine = new Line
            {
                X1 = line.Start.X,
                Y1 = line.Start.Y,
                X2 = line.End.X,
                Y2 = line.End.Y
            };
            roadLine.Style = (Style)canvas.Resources["RoadLine"];
            canvas.Children.Add(roadLine);
            
            // 绘制线段起点
            DrawPoint(line.Start, "PointStyle", out _);
            // 绘制线段终点
            DrawPoint(line.End, "PointStyle", out _);
        }
    }

    private void DrawPoint(Point point, string styleKey, out UIElement marker)
    {
        // 绘制点标记
        var ellipse = new Ellipse
        {
            Width = 12,
            Height = 12,
            Fill = Brushes.Green
        };
        ellipse.Style = (Style)canvas.Resources[styleKey];
        Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
        Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);
        canvas.Children.Add(ellipse);
        marker = ellipse;
    }

    private void DrawPath(List<Point> path)
    {
        // 绘制路径
        for (int i = 0; i < path.Count - 1; i++)
        {
            var line = new Line
            {
                X1 = path[i].X,
                Y1 = path[i].Y,
                X2 = path[i + 1].X,
                Y2 = path[i + 1].Y
            };
            line.Style = (Style)canvas.Resources["PathLine"];
            canvas.Children.Add(line);
            _pathElements.Add(line);
        }
    }

    private void canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        // 获取鼠标点击位置
        var point = e.GetPosition(canvas);
        var point2D = new Point(point.X, point.Y);

        if (_isSelectingStart)
        {
            // 选择起点
            if (_startMarker != null)
            {
                canvas.Children.Remove(_startMarker);
            }
            _startPoint = point2D;
            DrawPoint(point2D, "StartPointStyle", out _startMarker);
            _isSelectingStart = false;
        }
        else
        {
            // 选择终点
            if (_endMarker != null)
            {
                canvas.Children.Remove(_endMarker);
            }
            _endPoint = point2D;
            DrawPoint(point2D, "EndPointStyle", out _endMarker);
            _isSelectingStart = true;

            
        }

        UpdateStatus();
    }

    private void UpdateStatus()
    {
        if (_isSelectingStart)
        {
            txtStatus.Text = "请点击选择起点...";
        }
        else
        {
            txtStatus.Text = "请点击选择终点...";
        }
    }

    private void ClearPath()
    {
        // 清除路径和标记
        foreach (var element in _pathElements)
        {
            canvas.Children.Remove(element);
        }
        _pathElements.Clear();
    }

    private void btnClear_Click(object sender, RoutedEventArgs e)
    {
        // 清除所有标记和路径
        if (_startMarker != null)
        {
            canvas.Children.Remove(_startMarker);
            _startMarker = null;
        }
        if (_endMarker != null)
        {
            canvas.Children.Remove(_endMarker);
            _endMarker = null;
        }
        ClearPath();
        _startPoint = null;
        _endPoint = null;
        _isSelectingStart = true;
        UpdateStatus();
    }

    private void btnReset_Click(object sender, RoutedEventArgs e)
    {
        // 重置所有内容
        btnClear_Click(sender, e);
        canvas.Children.Clear();
        InitializeData();
        DrawMap();
    }

    private void btnTemp_Click(object sender, RoutedEventArgs e)
    {
        // 检查起点和终点是否已选择
        if (!_startPoint.HasValue || !_endPoint.HasValue)
        {
            txtStatus.Text = "请先选择起点和终点！";
            return;
        }
        
        // 创建新的图构建器
        var tempPlanner = new GraphBuilder();
        
        // 添加起点的临时点
        var startTempLines = tempPlanner.AddTemporaryPoint(_lines, _startPoint.Value);
        // 添加终点的临时点
        var endTempLines = tempPlanner.AddTemporaryPoint(_lines, _endPoint.Value);
        
        if (startTempLines == null || endTempLines == null)
        {
            txtStatus.Text = "无法添加临时点！";
            return;
        }
        
        // 获取需要替换的线段ID
        var idsToRemove = startTempLines.Select(x => x.Id).Concat(endTempLines.Select(x => x.Id)).ToList();
        
        // 更新线路列表
        _lines = _lines.Where(x => !idsToRemove.Contains(x.Id)).ToList();
        _lines.AddRange(startTempLines);
        _lines.AddRange(endTempLines);
        
        // 更新图构建器
        if (_planner == null)
        {
            _planner = new GraphBuilder();
        }
        _planner.Set(_lines);
        
        // 重新绘制地图
        DrawMap();
        txtStatus.Text = "临时点已添加，地图已更新！";
    }

    private void btnSort_Click(object sender, RoutedEventArgs e)
    {
        // 检查起点和终点是否已选择
        if (!_startPoint.HasValue || !_endPoint.HasValue)
        {
            txtStatus.Text = "请先选择起点和终点！";
            return;
        }
        
        // 检查_planner是否已初始化
        if (_planner == null)
        {
            txtStatus.Text = "系统错误：路径规划器未初始化！";
            return;
        }
        
        // 清除之前的路径
        ClearPath();

        // 查找最短路径
        var paths = _planner.FindShortestPath(_startPoint.Value, _endPoint.Value);

        if (paths.Count > 0)
        {
            // 绘制路径
            DrawPath(paths);
            txtStatus.Text = $"最短路径已找到，包含 {paths.Count} 个点！";
        }
        else
        {
            txtStatus.Text = "无法找到从起点到终点的路径！";
        }
    }
}