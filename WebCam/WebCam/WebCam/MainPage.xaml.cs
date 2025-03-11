using System;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Maui.Controls;

namespace WebCam
{
    public partial class MainPage : ContentPage
    {
        private readonly System.Net.Http.HttpClient _httpClient = new System.Net.Http.HttpClient();
        public MainPage()
        {
            InitializeComponent();

        }
        private async void trySendMessage(string jsondata) 
        {
            string url = "http://192.168.100.47:5000/receive_data"; // Reemplaza con la IP de tu PC

            var jsonData = $"{{\"message\": \"{jsondata}\"}}";// JSON que se enviará
            var content = new StringContent(jsonData, Encoding.UTF8, "application/json");

            try
            {
                HttpResponseMessage response = await _httpClient.PostAsync(url, content);
                string responseString = await response.Content.ReadAsStringAsync();
                //await DisplayAlert("Respuesta del servidor", responseString, "OK");
            }
            catch (Exception ex)
            {
                await DisplayAlert("Error", ex.Message, "OK");
            }
        }
   
        private async void OnUpButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;

            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original

            trySendMessage("1");
        }
        private async void OnLeftButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("2");
        }
        private async void OnRightButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("3");
        }
        private async void OnDownButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("4");
        }

    }

}
