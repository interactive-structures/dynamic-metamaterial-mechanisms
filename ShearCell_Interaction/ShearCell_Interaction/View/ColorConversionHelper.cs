using System.Windows.Media;
using ShearCell_Interaction.Helper;

namespace ShearCell_Interaction.View
{
    public class ColorConversionHelper
    {
        // credit to http://james-ramsden.com/convert-from-hsl-to-rgb-colour-codes-in-c/

        public static Color ColorFromHSL(double h, double s, double l)
        {
            double r = 0, g = 0, b = 0;

            //if (l != 0)
            if (MathHelper.IsUnequalDouble(l, 0))
            {
                //if (s == 0){
                if (MathHelper.IsEqualDouble(s, 0))
                {
                    r = g = b = l;
                }
                else
                {
                    double temp2;
                    if (l < 0.5)
                        temp2 = l*(1.0 + s);
                    else
                        temp2 = l + s - (l*s);

                    double temp1 = 2.0*l - temp2;

                    r = GetColorComponent(temp1, temp2, h + 1.0/3.0);
                    g = GetColorComponent(temp1, temp2, h);
                    b = GetColorComponent(temp1, temp2, h - 1.0/3.0);
                }
            }
            return Color.FromArgb(255, (byte)(255 * r), (byte)(255 * g), (byte)(255 * b));
        }

        private static double GetColorComponent(double temp1, double temp2, double temp3)
        {
            if (temp3 < 0.0)
                temp3 += 1.0;
            else if (temp3 > 1.0)
                temp3 -= 1.0;

            if (temp3 < 1.0 / 6.0)
                return temp1 + (temp2 - temp1) * 6.0 * temp3;
            if (temp3 < 0.5)
                return temp2;
            if (temp3 < 2.0 / 3.0)
                return temp1 + ((temp2 - temp1) * ((2.0 / 3.0) - temp3) * 6.0);
            return temp1;
        }


    }
}
