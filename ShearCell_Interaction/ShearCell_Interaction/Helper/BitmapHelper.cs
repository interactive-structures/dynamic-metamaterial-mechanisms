using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Helper
{
    class BitmapHelper
    {
        public static Shape GetDeepCopyOfShape(Shape shape)
        {
            var xaml = System.Windows.Markup.XamlWriter.Save(shape);
            var deepCopy = System.Windows.Markup.XamlReader.Parse(xaml) as Shape;
            return deepCopy;
        }

        public static void RenderBitmap(List<Shape> shapes, string filename, int canvasWidth = -1, int canvasHeight = -1, int offsetX = 0, int offsetY = 0, int scaleFactor = 1)
        {
            canvasWidth = canvasWidth > 0 ? canvasWidth : ViewModel.WindowWidth;
            canvasHeight = canvasHeight > 0 ? canvasHeight : ViewModel.WindowHeight;

            var canvas = new Canvas { Width = canvasWidth, Height = canvasHeight, Background = Brushes.White };
            foreach (var shape in shapes)
            {
                shape.Margin = new Thickness(shape.Margin.Left + offsetX, shape.Margin.Top - offsetY, shape.Margin.Right, shape.Margin.Bottom);
                canvas.Children.Add(shape);
            }

            var size = new Size(canvasWidth, canvasHeight);
            canvas.Measure(size);
            canvas.Arrange(new Rect(size));

            var bitmap = new RenderTargetBitmap(canvasWidth * scaleFactor, canvasHeight * scaleFactor, 96d * scaleFactor, 96d * scaleFactor, PixelFormats.Default);
            bitmap.Render(canvas);

            //BitmapEncoder encoder = new PngBitmapEncoder();
            BitmapEncoder encoder = new JpegBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));

            using (var stream = File.OpenWrite(filename))
                encoder.Save(stream);

            bitmap.Clear();
            bitmap = null;
            encoder = null;
        }

        public static void RenderBitmap(Canvas canvas, string filename, int canvasWidth = -1, int canvasHeight = -1)
        {
            canvasWidth = canvasWidth > 0 ? canvasWidth : (int)canvas.ActualWidth;
            canvasHeight = canvasHeight > 0 ? canvasHeight : (int)canvas.ActualHeight;

            var size = new Size(canvasWidth, canvasHeight);
            canvas.Measure(size);
            canvas.Arrange(new Rect(size));

            var bitmap = new RenderTargetBitmap(canvasWidth, canvasHeight, 96d, 96d, PixelFormats.Default);
            bitmap.Render(canvas);

            //BitmapEncoder encoder = new PngBitmapEncoder();
            BitmapEncoder encoder = new JpegBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));

            using (var stream = File.OpenWrite(filename))
                encoder.Save(stream);

            bitmap.Clear();
            bitmap = null;
            encoder = null;
        }
    }
}
