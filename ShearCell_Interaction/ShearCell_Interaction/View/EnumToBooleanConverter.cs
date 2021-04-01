using System;
using System.Windows;
using System.Windows.Data;

namespace ShearCell_Interaction.View
{
    public class EnumToBooleanConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if(value != null)
                return value.Equals(parameter);

            return DependencyProperty.UnsetValue;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value != null)
                return value.Equals(true) ? parameter : Binding.DoNothing;

            return DependencyProperty.UnsetValue;
        }
    }
}
