#include <gtk/gtk.h>
#include <thread>
#include <kinect_streamer/kinect_streamer.hpp>
#include <gdk-pixbuf/gdk-pixbuf.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>
#include <stdio.h>

using namespace cv;


typedef struct ButtonData {
    GtkWindow* window;
    GtkEntry* entry;
} ButtonData;

typedef struct EntryData {
    GtkEntry* entry;
} EntryData;

/* Create a new hbox with an image and a label packed into it
 * and return the box. */

static GtkBox* xpm_label_box(gchar* xpm_filename, gchar* label_text) {
    GtkBox* box;
    GtkLabel* label;
    GtkImage* image;

    /* Create box for image and label */
    box = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_set_border_width((GtkContainer*)box, 20);

    /* Now on to the image stuff */
    image = (GtkImage*)gtk_image_new_from_file(xpm_filename);

    /* Create a label for the button */
    label = (GtkLabel*)gtk_label_new(label_text);

    /* Pack the image and label into the box */
    gtk_box_pack_start(box, (GtkWidget*)image, FALSE, FALSE, 10);
    gtk_box_pack_start(box, (GtkWidget*)label, FALSE, FALSE, 10);

    gtk_widget_show((GtkWidget*)image);
    gtk_widget_show((GtkWidget*)label);

    return box;
}


static void callback_start(GtkWidget* widget, gpointer user_data) {
    EntryData* entry_data = (EntryData*)user_data;
    GtkEntry* entry = entry_data->entry;
    char* text = (char*)gtk_entry_get_text(entry);
    std::cout << text << std::endl;
    char* args[] = {"rosrun kinect_streamer_bin kinect_viewer_cli", text, NULL};
    execvp(args[0], args);
}

static void callback_select_folder(GtkWidget* widget, gpointer user_data) {
    GtkFileChooser* file_chooser;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
    ButtonData* button_data = (ButtonData*)user_data;
    GtkWindow* window = button_data->window;
    GtkEntry* entry = button_data->entry;
    gint res;
    file_chooser = (GtkFileChooser*)gtk_file_chooser_dialog_new("Select folder to read data",
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

    /* GtkWidget is the storage type for widgets */
    GtkBox* vbox;
    GtkBox* hbox;
    GtkWindow* window;
    GtkButton* button_folder;
    GtkButton* button_start;
    GtkBox* box_folder;
    GtkBox* box_start;
    GtkEntry* entry;
    GtkLabel* label_select;

    gtk_init (&argc, &argv);

    /* Create a new window */
    window = (GtkWindow*)gtk_window_new(GTK_WINDOW_TOPLEVEL);

    label_select = (GtkLabel*)gtk_label_new("Select a folder to store data:");

    entry = (GtkEntry*)gtk_entry_new();

    /* Create buttons*/
    button_folder = (GtkButton*)gtk_button_new();
    button_start = (GtkButton*)gtk_button_new();

    /* Set the window title */
    gtk_window_set_title(window, "Kinect Viewer GUI");

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
        gtk_entry_set_text(entry, current_path);
        free(current_path);
    }

    ButtonData button_data = (ButtonData){window, entry};
    EntryData entry_data = (EntryData){entry};

    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect(button_folder, "clicked", (GCallback)callback_select_folder, (gpointer)(&button_data));
    g_signal_connect(button_start, "clicked", (GCallback)callback_start, (gpointer)(&entry_data));

    /* This calls our box creating function */
    box_folder = xpm_label_box("folder.xpm", "Select Folder");
    box_start = xpm_label_box("folder.xpm", "Start");

    vbox = (GtkBox*)gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    hbox = (GtkBox*)gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);

    gtk_box_pack_start(hbox, (GtkWidget*)button_start, TRUE, TRUE, 10);

    gtk_box_pack_start(vbox, (GtkWidget*)label_select, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)button_folder, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)entry, FALSE, FALSE, 10);
    gtk_box_pack_start(vbox, (GtkWidget*)hbox, FALSE, FALSE, 10);

    gtk_container_add((GtkContainer*)window, (GtkWidget*)vbox);
    gtk_container_add((GtkContainer*)button_folder, (GtkWidget*)box_folder);
    gtk_container_add((GtkContainer*)button_start, (GtkWidget*)box_start);

    /* Pack and show all our widgets */

    gtk_widget_show((GtkWidget*)box_folder);
    gtk_widget_show((GtkWidget*)box_start);

    gtk_widget_show((GtkWidget*)vbox);
    gtk_widget_show((GtkWidget*)hbox);

    gtk_widget_show((GtkWidget*)label_select);
    
    gtk_widget_show((GtkWidget*)entry);

    gtk_widget_show((GtkWidget*)button_folder);
    gtk_widget_show((GtkWidget*)button_start);

    gtk_widget_show((GtkWidget*)window);

    /* Rest in gtk_main and wait for the fun to begin! */
    gtk_main();
    return 0;
}