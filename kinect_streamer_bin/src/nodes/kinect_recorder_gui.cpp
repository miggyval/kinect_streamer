#include <gtk/gtk.h>
#include <thread>
#include <kinect_streamer/kinect_streamer.hpp>
#include <gdk-pixbuf/gdk-pixbuf.h>


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>
#include <stdio.h>

using namespace cv;

typedef struct DeviceData {
    GtkComboBoxText* cbox;
} DeviceData;

typedef struct ButtonData {
    GtkWindow* window;
    GtkEntry* entry;
} ButtonData;

typedef struct EntryData {
    GtkEntry* entry_folder;
    GtkComboBoxText* cbox_device;
} EntryData;

/* Create a new hbox with an image and a label packed into it
 * and return the box. */

static GtkBox* xpm_label_box(gchar* xpm_filename, gchar* label_text) {
    GtkBox* box;
    GtkLabel* label;
    GtkImage* image;

    /* Create box for image and label */
    box = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_set_border_width((GtkContainer*)box, 2);

    /* Now on to the image stuff */
    image = (GtkImage*)gtk_image_new_from_file(xpm_filename);

    /* Create a label for the button */
    label = (GtkLabel*)gtk_label_new(label_text);

    /* Pack the image and label into the box */
    gtk_box_pack_start(box, (GtkWidget*)image, FALSE, FALSE, 2);
    gtk_box_pack_start(box, (GtkWidget*)label, FALSE, FALSE, 2);

    gtk_widget_show((GtkWidget*)image);
    gtk_widget_show((GtkWidget*)label);

    return box;
}

const std::string kinect_prefix = std::string("Kinect v2: ");

static void callback_refresh_devices(GtkWidget* widget, gpointer user_data) {
    DeviceData* device_data = (DeviceData*)user_data;
    GtkComboBoxText* cbox = device_data->cbox;
    libfreenect2::Freenect2 freenect2;
    gtk_combo_box_text_remove_all(cbox);
    int num_devices = freenect2.enumerateDevices();
    for (int i = 0; i < num_devices; i++) {
        std::string serial = kinect_prefix + freenect2.getDeviceSerialNumber(i);
        gtk_combo_box_text_append(cbox, NULL, serial.c_str());
    }
}

static void callback_start(GtkWidget* widget, gpointer user_data) {
    EntryData* entry_data = (EntryData*)user_data;
    GtkEntry* entry_folder = entry_data->entry_folder;
    GtkComboBoxText* cbox_device = entry_data->cbox_device;
    char* text_folder = (char*)gtk_entry_get_text(entry_folder);
    char* text_serial = (char*)gtk_combo_box_text_get_active_text(cbox_device);
    std::cout << "TEST!" << std::endl;
}

static void callback_select_folder(GtkWidget* widget, gpointer user_data) {
    GtkFileChooser* file_chooser;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
    ButtonData* button_data = (ButtonData*)user_data;
    GtkWindow* window = button_data->window;
    GtkEntry* entry = button_data->entry;
    gint res;
    file_chooser = (GtkFileChooser*)gtk_file_chooser_dialog_new("Select folder to save data",
                                      window,
                                      action,
                                      ("_Cancel"),
                                      GTK_RESPONSE_CANCEL,
                                      ("_Select Folder"),
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);
    gtk_file_chooser_set_current_folder(file_chooser, ".");
    res = gtk_dialog_run((GtkDialog*)file_chooser);
    if (res == GTK_RESPONSE_ACCEPT) {
        char* filename = gtk_file_chooser_get_filename(file_chooser);
        gtk_entry_set_text(entry, filename);
    } else if (res == GTK_RESPONSE_CANCEL) {
    }
    gtk_widget_destroy((GtkWidget*)file_chooser);
}

int main(int argc, char** argv) {
    libfreenect2::setGlobalLogger(NULL);
    bool use_kinect = true;

    /* GtkWidget is the storage type for widgets */
    GtkBox* vbox;
    GtkBox* hbox_folder;
    GtkBox* hbox_device;
    GtkBox* hbox_start;
    GtkWindow* window;
    GtkButton* button_folder;
    GtkButton* button_start;
    GtkButton* button_refresh;
    GtkButton* button_device;
    GtkBox* box_folder;
    GtkBox* box_device;
    GtkBox* box_start;
    GtkBox* box_refresh;
    GtkEntry* entry_folder;
    GtkComboBoxText* cbox_device;
    GtkListStore *store;
    GtkTreeView* tree_view;

    gtk_init (&argc, &argv);

    /* Create a new window */
    window = (GtkWindow*)gtk_window_new(GTK_WINDOW_TOPLEVEL);

    /* Create entries */
    entry_folder    = (GtkEntry*)gtk_entry_new();

    /* Create buttons*/
    button_folder   = (GtkButton*)gtk_button_new();
    button_device   = (GtkButton*)gtk_button_new();
    button_start    = (GtkButton*)gtk_button_new();
    button_refresh    = (GtkButton*)gtk_button_new();

    /* Create combo boxes */
    cbox_device = (GtkComboBoxText*)gtk_combo_box_text_new_with_entry();

    store = gtk_list_store_new(1, G_TYPE_STRING);

    tree_view = (GtkTreeView*)gtk_tree_view_new();

    GtkTreeIter iter;
    for (gint i = 0; i < 10; i++) {
        gchar* data = (gchar*)"Hello";
        gtk_list_store_append(store, &iter);
        gtk_list_store_set (store, &iter, 0, data, -1);
    }

    /* Set the window title */
    gtk_window_set_title(window, "Kinect Recorder GUI");

    /* Set the window size */
    gtk_window_set_default_size(window, 400, 300);

    /* It's a good idea to do this for all windows. */
    g_signal_connect((GtkWidget*)window, "destroy", (GCallback)gtk_main_quit, NULL);

    g_signal_connect((GtkWidget*)window, "delete-event", (GCallback)gtk_main_quit, NULL);

    /* Sets the border width of the window. */
    gtk_container_set_border_width((GtkContainer*)window, 20);
    
    char* current_path;
    char* ptr;
    if ((current_path = (char*)malloc((size_t)1024)) != NULL) {
        ptr = getcwd(current_path, (size_t)1024);
        gtk_entry_set_text(entry_folder, current_path);
        free(current_path);
    }
    
    ButtonData button_data = (ButtonData){window, entry_folder};
    EntryData entry_data = (EntryData){entry_folder, cbox_device};
    DeviceData device_data = (DeviceData){cbox_device};

    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect(button_folder, "clicked", (GCallback)callback_select_folder, (gpointer)(&button_data));
    g_signal_connect(button_start, "clicked", (GCallback)callback_start, (gpointer)(&entry_data));
    g_signal_connect(button_refresh, "clicked", (GCallback)callback_refresh_devices, (gpointer)(&device_data));

    /* This calls our box creating function */
    box_folder = xpm_label_box("folder.xpm", "Select Folder");
    box_device = xpm_label_box("folder.xpm", "Add Device");
    box_start = xpm_label_box("folder.xpm", "Start");
    box_refresh = xpm_label_box("folder.xpm", "Refresh");

    vbox = (GtkBox*)gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

    hbox_folder = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    hbox_device = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    hbox_start  = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);

    gtk_box_pack_start(hbox_folder, (GtkWidget*)button_folder, FALSE, FALSE, 10);
    gtk_box_pack_start(hbox_folder, (GtkWidget*)entry_folder, FALSE, FALSE, 10);


    gtk_box_pack_start(hbox_device, (GtkWidget*)button_device, FALSE, FALSE, 10);
    gtk_box_pack_start(hbox_device, (GtkWidget*)cbox_device, FALSE, FALSE, 10);

    gtk_box_pack_start(hbox_start, (GtkWidget*)button_start, FALSE, FALSE, 10);
    gtk_box_pack_start(hbox_start, (GtkWidget*)button_refresh, FALSE, FALSE, 10);

    gtk_box_pack_start(vbox, (GtkWidget*)hbox_folder, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)hbox_device, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)hbox_start, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)tree_view, FALSE, FALSE, 10);

    gtk_container_add((GtkContainer*)window, (GtkWidget*)vbox);
    gtk_container_add((GtkContainer*)button_folder, (GtkWidget*)box_folder);
    gtk_container_add((GtkContainer*)button_start, (GtkWidget*)box_start);
    gtk_container_add((GtkContainer*)button_refresh, (GtkWidget*)box_refresh);
    gtk_container_add((GtkContainer*)button_device, (GtkWidget*)box_device);

    gtk_tree_view_set_model(tree_view, (GtkTreeModel*)store);

    /* Pack and show all our widgets */

    gtk_widget_show((GtkWidget*)box_folder);
    gtk_widget_show((GtkWidget*)box_device);
    gtk_widget_show((GtkWidget*)box_start);
    gtk_widget_show((GtkWidget*)box_refresh);

    gtk_widget_show((GtkWidget*)vbox);

    gtk_widget_show((GtkWidget*)hbox_folder);
    gtk_widget_show((GtkWidget*)hbox_device);
    gtk_widget_show((GtkWidget*)hbox_start);

    
    gtk_widget_show((GtkWidget*)entry_folder);

    gtk_widget_show((GtkWidget*)button_device);
    gtk_widget_show((GtkWidget*)button_folder);
    gtk_widget_show((GtkWidget*)button_start);
    gtk_widget_show((GtkWidget*)button_refresh);

    gtk_widget_show((GtkWidget*)cbox_device);

    gtk_widget_show((GtkWidget*)tree_view);

    gtk_widget_show((GtkWidget*)window);

    /* Rest in gtk_main and wait for the fun to begin! */
    gtk_main();
    return 0;
}